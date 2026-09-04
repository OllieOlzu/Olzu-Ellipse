/* =============================================================================
 * Voice-Controlled SD Card Music Player
 * -----------------------------------------------------------------------------
 * Target hardware : Seeed XIAO ESP32S3
 *                    INMP441   I2S microphone   (I2S0, RX)
 *                    MAX98357A I2S class-D amp  (I2S1, TX)
 *                    MicroSD card module        (SPI)
 *                    Onboard yellow LED          GPIO21, active-low
 *                    External LED strip          GPIO1 (D0), PWM-faded
 * Framework        : ESP-IDF v5.x, ESP-ADF (audio_pipeline), ESP-SR (MultiNet, English)
 *
 * Voice model
 * -----------
 * A single ESP-SR MultiNet (English) instance recognizes every phrase below
 * — no WakeNet model is loaded. Three wake-word phrases (WAKE_WORD_1/2/3
 * below, default "HEY SPEAKER" / "WAKE UP" / "COMPUTER") are treated as
 * equivalent by an application-level state machine in voice_task(): hearing
 * any one of them opens a command-listening window.
 *
 *   wake word    -> hearing any of WAKE_WORD_1/2/3 turns the onboard LED on,
 *                   fades the D0 strip LED in over LED_FADE_TIME, ducks the
 *                   music down to LISTENING_VOLUME so it's still just
 *                   audible in the background, and opens a command window
 *                   (LISTEN_WINDOW_US, currently 7 s).
 *   PLAY         -> resume the current track from where it left off
 *   STOP         -> pause; remembers the exact position, then feeds the amp
 *                   STOP_SILENCE_MS of real silence (see note below)
 *   BACK         -> jump to the previous track (by sorted position, not by
 *                   literal file number — see track-list notes below) and
 *                   play it from the start
 *   NEXT         -> jump to the next track and play it from the start
 *   RESTART      -> jump to the lowest-numbered track and play it from the start
 *   VOLUME DOWN  -> volume -= VOLUME_CHANGE
 *   VOLUME UP    -> volume += VOLUME_CHANGE
 *
 * Whichever of the above is recognized (or if the window simply times out
 * with nothing recognized), the music is restored to whatever volume it was
 * at *before* the wake word — then, if a real command came in, that command
 * is applied on top of the restored volume. So "VOLUME UP" raises the
 * volume you were already listening at, not LISTENING_VOLUME.
 *
 * Both LEDs turn off the instant a valid command is recognized, or when the
 * window expires, whichever happens first — the onboard LED switches off
 * immediately, the D0 strip LED fades out over LED_FADE_TIME. Detection
 * never blocks (no vTaskDelay in the recognition loop), so a wake word and
 * a command can be spoken with very little gap between them.
 *
 * MAX98357A silence handling
 * ---------------------------
 * The I2S TX peripheral repeats whatever was in its last DMA buffer(s) once
 * it runs out of freshly-written audio, rather than going quiet — the
 * MAX98357A just faithfully plays whatever it's handed. Real silence is
 * therefore written into the TX ring explicitly at the moments new audio is
 * about to stop flowing — see i2s_write_silence_ms() and its call sites in
 * player_open_track() and the CMD_STOP handler.
 *
 * Track numbering and supported formats
 * ---------------------------------------
 * Tracks are named "<number>.wav" or "<number>.mp3" on the SD card (1.mp3,
 * 2.wav, 3.mp3, ...) — the two formats can be freely mixed. Numbers may skip
 * (e.g. 1.mp3, 3.wav, 8.mp3 exist but 2,4-7 don't). At boot, the SD card
 * root directory is scanned once and every valid "<number>.wav"/"<number>.mp3"
 * file is recorded into a sorted list. NEXT/BACK/RESTART all operate on
 * *positions in that sorted list*, not on the literal file numbers, so gaps
 * are handled automatically: NEXT always goes to the next highest number
 * that actually exists, BACK to the next lowest, RESTART to the lowest one
 * present, and all three wrap around at the ends of the list.
 *
 * Playback goes through an ESP-ADF audio_pipeline:
 *
 *     [fatfs reader] -> [mp3_decoder or wav_decoder] -> [rsp_filter] -> [raw_stream]
 *
 * "rsp_filter" resamples and, if needed, downmixes whatever the decoder
 * reports for the current file — any sample rate, mono or stereo — down to
 * a fixed 44.1 kHz mono stream (PLAYBACK_SAMPLE_RATE below), which is what
 * raw_stream hands back to playback_task one chunk at a time via
 * raw_stream_read(). playback_task then applies volume and writes the
 * result straight to the MAX98357A with i2s_channel_write(). Source files
 * don't need to be pre-converted to any particular rate, bit depth or
 * channel count — an MP3 or WAV ripped straight from anywhere can just be
 * dropped on the card.
 *
 * Only the decoder element differs between an MP3 and a WAV track; the
 * reader, resample filter and raw sink are created once at boot and stay
 * put for the life of the program. When the next track needs the other
 * decoder, player_open_track() swaps it in with
 * audio_pipeline_breakup_elements()/audio_pipeline_relink().
 *
 * main/CMakeLists.txt's PRIV_REQUIRES needs audio_pipeline, audio_stream,
 * mp3_decoder, wav_decoder, rsp_filter, audio_sal and fatfs (or just
 * "esp-adf-libs" if your ESP-ADF checkout bundles the individual codec
 * components into that single component instead).
 *
 * The microphone path (INMP441 -> MultiNet) runs at 16 kHz, which is what
 * ESP-SR's MultiNet expects and is unrelated to the 44.1 kHz playback path
 * described above.
 *
 * Required one-time project configuration:
 *   idf.py menuconfig -> Component config -> ESP System Settings ->
 *     Channel for console output -> "USB Serial JTAG Controller"
 * This frees GPIO43/44 (D6/D7) from the default UART0 console assignment —
 * D6 carries the amplifier's DOUT line and D7 carries the SD card's MISO
 * line, both used below. Flashing and monitoring continue to work over the
 * same USB-C port.
 *
 * ESP-ADF's audio_stream component always pulls in a board definition
 * (components/audio_board), even though this file never calls into it — the
 * mic and speaker I2S channels are driven entirely by hand below, so
 * ESP-ADF never touches that hardware directly. The stock default
 * (Audio HAL -> Audio board -> "ESP32-Lyrat V4.3" in menuconfig) is fine to
 * leave selected; its pin values are simply never used by anything here.
 * ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "model_path.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"

/* ESP-ADF: audio_pipeline/audio_element core, plus the specific elements the
 * playback pipeline described above is built from. */
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "fatfs_stream.h"
#include "raw_stream.h"
#include "mp3_decoder.h"
#include "wav_decoder.h"
#include "filter_resample.h"

static const char *TAG = "voice_player";

/* -----------------------------------------------------------------------
 *  Pin map — Seeed XIAO ESP32S3
 *  D-number / GPIO pairs confirmed against Seeed's official pinout table.
 *  All 11 header pins (D0-D10) are used.
 * ----------------------------------------------------------------------- */

// INMP441 microphone -- I2S0, RX
#define MIC_WS_IO     GPIO_NUM_2    // D1
#define MIC_SCK_IO    GPIO_NUM_3    // D2
#define MIC_SD_IO     GPIO_NUM_4    // D3

// MAX98357A amplifier -- I2S1, TX (separate I2S peripheral from the mic)
#define SPK_BCLK_IO   GPIO_NUM_6    // D5
#define SPK_WS_IO     GPIO_NUM_5    // D4
#define SPK_DOUT_IO   GPIO_NUM_43   // D6 (requires UART0 console disabled, see header notes)

// MicroSD card -- SPI bus
#define SD_SCK_IO     GPIO_NUM_7    // D8
#define SD_MISO_IO    GPIO_NUM_44   // D7 (requires UART0 console disabled, see header notes)
#define SD_MOSI_IO    GPIO_NUM_8    // D9
#define SD_CS_IO      GPIO_NUM_9    // D10

// Onboard user LED (yellow), active-low
#define LED_GPIO      GPIO_NUM_21

// External LED strip (standard on/off strip, not addressable), PWM-faded
#define STRIP_LED_GPIO GPIO_NUM_1    // D0

#define SD_MOUNT_POINT "/sdcard"
#define SD_SPI_HOST    SPI2_HOST

/* -----------------------------------------------------------------------
 *  Wake words -- any one of the three opens a command-listening window.
 *  Edit these freely; MultiNet matches English phrases directly, no
 *  additional model training is needed.
 * ----------------------------------------------------------------------- */
#define WAKE_WORD_1   "ELIPS"
#define WAKE_WORD_2   "AHLIPS"
#define WAKE_WORD_3   "ILLIPS"

/* -----------------------------------------------------------------------
 *  Voice command IDs registered with MultiNet
 * ----------------------------------------------------------------------- */
#define CMD_WAKE1_ID      1
#define CMD_WAKE2_ID      2
#define CMD_WAKE3_ID      3
#define CMD_PLAY_ID       4
#define CMD_STOP_ID       5
#define CMD_BACK_ID       6
#define CMD_NEXT_ID       7
#define CMD_RESTART_ID    8
#define CMD_VOL_DOWN_ID   9
#define CMD_VOL_UP_ID     10

#define LISTEN_WINDOW_US       (7 * 1000 * 1000)  // 7 s LED "listening" window -- gives
                                                   // "VOLUME UP"/"VOLUME DOWN" a comfortable
                                                   // margin to be spoken in full without timing out.
#define MN_CREATE_DURATION_MS  7000                // MultiNet's own internal backstop timer,
                                                    // matched to LISTEN_WINDOW_US above.

/* -----------------------------------------------------------------------
 *  Player tunables
 * ----------------------------------------------------------------------- */
static const float VOLUME_CHANGE = 0.1f;   // step size for "VOLUME UP" / "VOLUME DOWN"

#define INITIAL_VOLUME      0.3f
#define MIN_VOLUME          0.0f
#define MAX_VOLUME          1.0f
#define CLIP_MIN            (-12000)
#define CLIP_MAX            (12000)

// Volume the player ducks to while listening for a command after the wake
// word, so the music stays just barely audible instead of competing with
// the mic for the user's command. Restored once a command is heard or the
// listening window times out -- see CMD_DUCK_VOLUME / CMD_RESTORE_VOLUME.
#define LISTENING_VOLUME    0.05f

// How long to feed the MAX98357A real silence for when STOP is heard,
// instead of just halting playback -- see i2s_write_silence_ms().
#define STOP_SILENCE_MS      250

// Silence flushed into the TX ring right before opening a different track
// (BACK/NEXT/RESTART/first PLAY), so nothing left over from the previous
// track is sitting there for the peripheral to loop on during the restart.
#define TRACK_SWITCH_FLUSH_MS 100

#define AUDIO_CHUNK_BYTES     4096   // ~46 ms of audio @ 44.1 kHz/16-bit/mono per loop iteration
#define MAX_TRACKS            200

// rsp_filter resamples/downmixes every track to this rate; i2s_speaker_init()
// below clocks the I2S peripheral to match.
#define PLAYBACK_SAMPLE_RATE  44100

// playback_task and every ESP-ADF pipeline element task run on this core;
// voice_task (mic capture + MultiNet inference) runs on the other one, so
// playback and voice recognition never compete for CPU time on the same core.
#define PLAYBACK_CORE  1
#define VOICE_CORE     0

/* -----------------------------------------------------------------------
 *  External LED strip -- PWM fade via the LEDC hardware peripheral. Fades
 *  are driven by hardware/ISR once started, so they never block voice_task
 *  or playback_task.
 * ----------------------------------------------------------------------- */
#define STRIP_LED_PWM_TIMER      LEDC_TIMER_0
#define STRIP_LED_PWM_CHANNEL    LEDC_CHANNEL_0
#define STRIP_LED_PWM_MODE       LEDC_LOW_SPEED_MODE
#define STRIP_LED_PWM_RES        LEDC_TIMER_10_BIT        // duty range 0-1023
#define STRIP_LED_PWM_FREQ_HZ    5000
#define STRIP_LED_PWM_MAX_DUTY   ((1 << STRIP_LED_PWM_RES) - 1)

#define LED_FADE_TIME            500   // ms -- fade in/out duration for the strip LED

typedef enum {
    CMD_NONE = 0,
    CMD_PLAY,
    CMD_STOP,
    CMD_BACK,
    CMD_NEXT,
    CMD_RESTART,
    CMD_VOL_UP,
    CMD_VOL_DOWN,
    CMD_DUCK_VOLUME,     // entered the post-wake-word listening window
    CMD_RESTORE_VOLUME,  // listening window ended (command heard, or timed out)
} player_cmd_t;

static QueueHandle_t s_cmd_queue;
static i2s_chan_handle_t s_rx_handle = NULL;   // microphone
static i2s_chan_handle_t s_tx_handle = NULL;   // speaker

/* ============================== Track list ============================== */

typedef enum { TRACK_EXT_WAV, TRACK_EXT_MP3 } track_ext_t;

typedef struct {
    int        number;   // the "<number>" part of the filename
    track_ext_t ext;
} track_entry_t;

typedef struct {
    track_entry_t entries[MAX_TRACKS];
    int count;
} track_list_t;

static track_list_t s_tracks = { .count = 0 };

static int track_entry_cmp(const void *a, const void *b) {
    return ((const track_entry_t *)a)->number - ((const track_entry_t *)b)->number;
}

// Fills *number/*ext and returns true if `name` matches exactly
// "<digits>.wav" or "<digits>.mp3" (case-insensitive extension); returns
// false otherwise. Rejects anything with trailing junk after the extension
// or no digits at all.
static bool parse_track_filename(const char *name, int *number, track_ext_t *ext) {
    int i = 0;
    long value = 0;
    int digits = 0;
    while (name[i] >= '0' && name[i] <= '9') {
        if (digits >= 9) return false;  // implausibly long number, not a real track
        value = value * 10 + (name[i] - '0');
        i++;
        digits++;
    }
    if (digits == 0) return false;
    if (strcasecmp(&name[i], ".wav") == 0) {
        *number = (int)value;
        *ext = TRACK_EXT_WAV;
        return true;
    }
    if (strcasecmp(&name[i], ".mp3") == 0) {
        *number = (int)value;
        *ext = TRACK_EXT_MP3;
        return true;
    }
    return false;
}

static void scan_tracks(void) {
    s_tracks.count = 0;
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Could not open %s to scan for tracks", SD_MOUNT_POINT);
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        int number;
        track_ext_t ext;
        if (parse_track_filename(entry->d_name, &number, &ext) && s_tracks.count < MAX_TRACKS) {
            s_tracks.entries[s_tracks.count].number = number;
            s_tracks.entries[s_tracks.count].ext = ext;
            s_tracks.count++;
        }
    }
    closedir(dir);
    qsort(s_tracks.entries, s_tracks.count, sizeof(track_entry_t), track_entry_cmp);

    if (s_tracks.count == 0) {
        ESP_LOGW(TAG, "No '<number>.wav' or '<number>.mp3' files found on SD card");
    } else {
        ESP_LOGI(TAG, "Found %d track(s) on SD card:", s_tracks.count);
        for (int i = 0; i < s_tracks.count; i++) {
            ESP_LOGI(TAG, "  position %d -> %d.%s", i, s_tracks.entries[i].number,
                     s_tracks.entries[i].ext == TRACK_EXT_MP3 ? "mp3" : "wav");
        }
    }
}

// Position in the sorted list, wrapping. These are what BACK/NEXT use, so
// gaps in numbering (1, 3, 8 ...) are followed correctly in both directions.
static int next_index(int index) { return (index + 1) % s_tracks.count; }
static int prev_index(int index) { return (index - 1 + s_tracks.count) % s_tracks.count; }

/* ====================== Player state (owned by playback_task only) ====================== */
// Everything below is touched exclusively from playback_task. Commands
// arrive only via the queue, so there is no shared-state race and no mutex
// is needed anywhere in this file.

typedef struct {
    int   track_index;    // position in s_tracks.entries[]
    bool  track_loaded;   // true once the pipeline has been successfully primed for track_index
    bool  playing;
    float volume;
    bool  ducked;         // true while volume == LISTENING_VOLUME (see CMD_DUCK_VOLUME)
    float saved_volume;   // volume to restore to once CMD_RESTORE_VOLUME arrives
} player_state_t;

static player_state_t s_player = {
    .track_index = 0,
    .track_loaded = false,
    .playing = false,
    .volume = INITIAL_VOLUME,
    .ducked = false,
    .saved_volume = INITIAL_VOLUME,
};

static void track_path(int index, char *buf, size_t buf_len) {
    const track_entry_t *t = &s_tracks.entries[index];
    snprintf(buf, buf_len, "%s/%d.%s", SD_MOUNT_POINT, t->number, t->ext == TRACK_EXT_MP3 ? "mp3" : "wav");
}

// Writes `ms` milliseconds of silence to the speaker. The I2S TX peripheral
// repeats whatever was in its last DMA buffer once it runs out of freshly-
// written audio instead of going quiet, so silence is written explicitly
// here at track transitions and on STOP to prevent audible looping.
static void i2s_write_silence_ms(uint32_t ms) {
    static const uint8_t silence[256] = {0}; // 16-bit samples -> always write an even byte count
    size_t total_bytes = (size_t)(((uint64_t)PLAYBACK_SAMPLE_RATE * 2 * ms) / 1000);
    size_t done = 0;
    while (done < total_bytes) {
        size_t want = total_bytes - done;
        if (want > sizeof(silence)) want = sizeof(silence);
        size_t written = 0;
        esp_err_t err = i2s_channel_write(s_tx_handle, silence, want, &written, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Speaker write failed (%s) -- check amplifier wiring "
                          "(BCLK/WS/DOUT to D5/D4/D6)", esp_err_to_name(err));
            break; // don't spin forever if the driver reports trouble
        }
        if (written == 0) break;
        done += written;
    }
}

/* ============================== ESP-ADF playback pipeline ==============================
 *
 *   [fatfs reader] -> [mp3_decoder or wav_decoder] -> [rsp_filter] -> [raw_stream]
 *
 * playback_task pulls decoded/resampled/downmixed 16-bit mono PCM out of the
 * raw_stream end with raw_stream_read() and writes it to the speaker,
 * applying volume and the silence-flush workaround above in playback_task
 * itself. Only the decoder element linked into the pipeline changes between
 * tracks (see player_open_track()); the reader, filter and sink are created
 * once here and stay put.
 */
static audio_pipeline_handle_t    s_pipeline;
static audio_event_iface_handle_t s_pipeline_evt;
static audio_element_handle_t     s_fatfs_el;
static audio_element_handle_t     s_mp3_el;
static audio_element_handle_t     s_wav_el;
static audio_element_handle_t     s_rsp_el;
static audio_element_handle_t     s_raw_el;
static track_ext_t                s_active_ext;   // which decoder is currently linked into the pipeline

static void player_pipeline_init(void) {
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_cfg.task_core = PLAYBACK_CORE;
    s_fatfs_el = fatfs_stream_init(&fatfs_cfg);

    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_cfg.task_core = PLAYBACK_CORE;
    mp3_cfg.stack_in_ext = false;   // avoid xTaskCreateRestrictedPinnedToCore path (needs unapplied IDF patch)
    s_mp3_el = mp3_decoder_init(&mp3_cfg);

    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    wav_cfg.task_core = PLAYBACK_CORE;
    wav_cfg.stack_in_ext = false;   // avoid xTaskCreateRestrictedPinnedToCore path (needs unapplied IDF patch)
    s_wav_el = wav_decoder_init(&wav_cfg);

    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.task_core = PLAYBACK_CORE;
    rsp_cfg.stack_in_ext = false;   // avoid xTaskCreateRestrictedPinnedToCore path (needs unapplied IDF patch)
    rsp_cfg.src_rate  = PLAYBACK_SAMPLE_RATE; // placeholder -- corrected per track, see AEL_MSG_CMD_REPORT_MUSIC_INFO below
    rsp_cfg.src_ch    = 1;
    rsp_cfg.dest_rate = PLAYBACK_SAMPLE_RATE; // fixed: matches the clock i2s_speaker_init() sets up below
    rsp_cfg.dest_bits = 16;
    rsp_cfg.dest_ch   = 1;                    // fixed: always downmix to mono for the one MAX98357A speaker
    s_rsp_el = rsp_filter_init(&rsp_cfg);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER; // playback_task reads the final PCM out of this end
    s_raw_el = raw_stream_init(&raw_cfg);
    // Must be short, not the framework's default of "wait forever": playback_task
    // also has to poll s_cmd_queue and pipeline events every time round its loop,
    // so a voice command is never left waiting behind a slow/idle read.
    audio_element_set_input_timeout(s_raw_el, pdMS_TO_TICKS(50));

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    s_pipeline = audio_pipeline_init(&pipeline_cfg);
    audio_pipeline_register(s_pipeline, s_fatfs_el, "file");
    audio_pipeline_register(s_pipeline, s_mp3_el,   "mp3");
    audio_pipeline_register(s_pipeline, s_wav_el,   "wav");
    audio_pipeline_register(s_pipeline, s_rsp_el,   "rsp");
    audio_pipeline_register(s_pipeline, s_raw_el,   "raw");

    // Arbitrary starting choice -- player_open_track() transparently relinks
    // to "mp3" instead the first time it's actually asked to open one.
    const char *initial_link[4] = {"file", "wav", "rsp", "raw"};
    audio_pipeline_link(s_pipeline, initial_link, 4);
    s_active_ext = TRACK_EXT_WAV;

    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    s_pipeline_evt = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(s_pipeline, s_pipeline_evt);
}

// Opens s_tracks.entries[index], swapping in whichever decoder its extension
// needs if that's not already the one linked into the pipeline, then starts
// the pipeline running from the first audio frame. Leaves s_player.playing
// untouched; the caller decides whether to start.
static bool player_open_track(int index) {
    if (s_tracks.count == 0) {
        ESP_LOGW(TAG, "No tracks available on SD card");
        s_player.track_loaded = false;
        return false;
    }

    // Flush silence into the TX ring before touching the SD card. Opening a
    // track takes a variable amount of time (FAT lookups, a slow card, MP3
    // needing a few frames to lock on), and nothing writes new audio until
    // that finishes. Runs on every track switch (BACK/NEXT/RESTART/first
    // PLAY/auto-advance), since they all share this same gap.
    i2s_write_silence_ms(TRACK_SWITCH_FLUSH_MS);

    track_ext_t needed = s_tracks.entries[index].ext;

    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    audio_pipeline_terminate(s_pipeline);

    if (needed != s_active_ext) {
        audio_element_handle_t outgoing = (s_active_ext == TRACK_EXT_MP3) ? s_mp3_el : s_wav_el;
        audio_pipeline_breakup_elements(s_pipeline, outgoing);
        const char *tags[4] = {"file", (needed == TRACK_EXT_MP3) ? "mp3" : "wav", "rsp", "raw"};
        audio_pipeline_relink(s_pipeline, tags, 4);
        // audio_pipeline_breakup_elements() drops every element's registered
        // listener as a side effect, so it must always be re-set after a relink.
        audio_pipeline_set_listener(s_pipeline, s_pipeline_evt);
        s_active_ext = needed;
    }

    char path[64];
    track_path(index, path, sizeof(path));
    audio_element_set_uri(s_fatfs_el, path);
    audio_pipeline_reset_ringbuffer(s_pipeline);
    audio_pipeline_reset_elements(s_pipeline);

    if (audio_pipeline_run(s_pipeline) != ESP_OK) {
        ESP_LOGE(TAG, "Could not start pipeline for %s", path);
        s_player.track_loaded = false;
        return false;
    }

    s_player.track_index  = index;
    s_player.track_loaded = true;
    ESP_LOGI(TAG, "Opened %s", path);
    return true;
}

// Used when auto-advancing (end of track, or a decode/open error reported by
// the pipeline -- see playback_task) so that a single bad or corrupt file on
// the card can't get the player stuck retrying it forever: tries at most
// once per track in the list before giving up.
static void player_advance_and_play(int start_index) {
    int idx = start_index;
    for (int attempts = 0; attempts < s_tracks.count; attempts++) {
        if (player_open_track(idx)) {
            s_player.playing = true;
            return;
        }
        idx = next_index(idx);
    }
    ESP_LOGE(TAG, "No playable tracks found after trying all %d entries", s_tracks.count);
    s_player.playing = false;
}

static void player_apply_volume(uint8_t *buf, size_t n_bytes) {
    for (size_t i = 0; i + 1 < n_bytes; i += 2) {
        int16_t sample;
        memcpy(&sample, &buf[i], 2);
        int32_t scaled = (int32_t)((float)sample * s_player.volume);
        if (scaled > CLIP_MAX) scaled = CLIP_MAX;
        if (scaled < CLIP_MIN) scaled = CLIP_MIN;
        int16_t out = (int16_t)scaled;
        memcpy(&buf[i], &out, 2);
    }
}

/* ============================== Playback task ============================== */

static void playback_task(void *arg) {
    uint8_t *chunk = malloc(AUDIO_CHUNK_BYTES);
    assert(chunk);
    player_cmd_t cmd;
    audio_event_iface_msg_t msg;

    while (1) {
        // While paused/stopped there is nothing to stream, so block on the
        // queue to use ~0% CPU. While playing, poll without blocking so a
        // command can interrupt mid-stream with low latency.
        TickType_t wait = s_player.playing ? 0 : portMAX_DELAY;

        if (xQueueReceive(s_cmd_queue, &cmd, wait) == pdTRUE) {
            switch (cmd) {
            case CMD_PLAY:
                if (!s_player.playing) {
                    s_player.playing = s_player.track_loaded ? true : player_open_track(s_player.track_index);
                }
                break;

            case CMD_STOP:
                // Just stop draining the pipeline -- fatfs_stream, the active
                // decoder and rsp_filter all block on their own full ring
                // buffers once raw_stream's is no longer being emptied, using
                // ~0% CPU while paused this way, and every bit of their
                // internal state (including the file position) is left
                // exactly where it was, so PLAY resumes from precisely this
                // point.
                s_player.playing = false;
                // Without this, the MAX98357A just keeps looping whatever
                // was last in the TX ring, audibly repeating it forever.
                i2s_write_silence_ms(STOP_SILENCE_MS);
                break;

            case CMD_BACK:
                if (s_tracks.count > 0) {
                    s_player.playing = player_open_track(prev_index(s_player.track_index));
                }
                break;

            case CMD_NEXT:
                if (s_tracks.count > 0) {
                    s_player.playing = player_open_track(next_index(s_player.track_index));
                }
                break;

            case CMD_RESTART:
                if (s_tracks.count > 0) {
                    s_player.playing = player_open_track(0); // position 0 == lowest-numbered track
                }
                break;

            case CMD_VOL_UP:
                s_player.volume += VOLUME_CHANGE;
                if (s_player.volume > MAX_VOLUME) s_player.volume = MAX_VOLUME;
                ESP_LOGI(TAG, "Volume -> %.2f", s_player.volume);
                break;

            case CMD_VOL_DOWN:
                s_player.volume -= VOLUME_CHANGE;
                if (s_player.volume < MIN_VOLUME) s_player.volume = MIN_VOLUME;
                ESP_LOGI(TAG, "Volume -> %.2f", s_player.volume);
                break;

            case CMD_DUCK_VOLUME:
                // Wake word just fired: drop to LISTENING_VOLUME so the
                // music is still just barely audible while we wait for a
                // command. Only save the pre-wake volume the *first* time --
                // if the wake word is repeated mid-window (see voice_task),
                // we're already ducked and must not clobber saved_volume
                // with LISTENING_VOLUME itself.
                if (!s_player.ducked) {
                    s_player.saved_volume = s_player.volume;
                    s_player.volume = LISTENING_VOLUME;
                    s_player.ducked = true;
                    ESP_LOGI(TAG, "Listening -- volume %.2f -> %.2f", s_player.saved_volume, s_player.volume);
                }
                break;

            case CMD_RESTORE_VOLUME:
                // Listening window ended, either because a command was
                // recognized or because it timed out. Either way, restore
                // the volume the music was at before the wake word -- this
                // always runs *before* the actual command (if any) is
                // applied, since voice_task queues them in that order.
                if (s_player.ducked) {
                    s_player.volume = s_player.saved_volume;
                    s_player.ducked = false;
                    ESP_LOGI(TAG, "Volume restored -> %.2f", s_player.volume);
                }
                break;

            default:
                break;
            }
        }

        // Service pipeline events: adapt rsp_filter to whatever sample
        // rate/channel count the currently-active decoder just reported for
        // this track (fires once near the start of every track, MP3 or WAV,
        // whether or not the decoder element itself changed), and catch
        // open/decode errors so a single bad file on the card can't
        // silently wedge playback.
        if (audio_event_iface_listen(s_pipeline_evt, &msg, 0) == ESP_OK) {
            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO &&
                (msg.source == (void *)s_mp3_el || msg.source == (void *)s_wav_el)) {
                audio_element_info_t info = {0};
                audio_element_getinfo((audio_element_handle_t)msg.source, &info);
                ESP_LOGI(TAG, "Track format: %d Hz, %d ch, %d-bit", info.sample_rates, info.channels, info.bits);
                if (info.bits != 16) {
                    // player_apply_volume() and the rest of playback_task assume
                    // 16-bit samples throughout -- anything else (e.g. 24-bit
                    // PCM WAVs) comes out the other end as noise rather than
                    // audio. Bail out of this track instead of playing it.
                    ESP_LOGE(TAG, "Track %d is %d-bit, only 16-bit is supported -- skipping it",
                             s_player.track_index, info.bits);
                    if (s_tracks.count > 0) {
                        player_advance_and_play(next_index(s_player.track_index));
                    } else {
                        s_player.playing = false;
                    }
                } else {
                    rsp_filter_set_src_info(s_rsp_el, info.sample_rates, info.channels);
                }
            } else if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
                       (msg.source == (void *)s_fatfs_el || msg.source == (void *)s_mp3_el ||
                        msg.source == (void *)s_wav_el  || msg.source == (void *)s_rsp_el) &&
                       ((int)msg.data == AEL_STATUS_ERROR_OPEN    || (int)msg.data == AEL_STATUS_ERROR_INPUT ||
                        (int)msg.data == AEL_STATUS_ERROR_PROCESS || (int)msg.data == AEL_STATUS_ERROR_OUTPUT)) {
                ESP_LOGE(TAG, "Pipeline reported an error (status=%d) on track %d -- skipping it",
                         (int)msg.data, s_player.track_index);
                if (s_tracks.count > 0) {
                    player_advance_and_play(next_index(s_player.track_index));
                } else {
                    s_player.playing = false;
                }
            }
        }

        if (s_player.playing) {
            int r = raw_stream_read(s_raw_el, (char *)chunk, AUDIO_CHUNK_BYTES);
            if (r > 0) {
                player_apply_volume(chunk, (size_t)r);
                size_t written = 0;
                i2s_channel_write(s_tx_handle, chunk, (size_t)r, &written, portMAX_DELAY);
            } else if (r == AEL_IO_TIMEOUT) {
                // Nothing decoded yet (e.g. right after opening a track, or a
                // slow SD-card read) -- not an error, just try again next
                // time round the loop.
            } else {
                // AEL_IO_DONE (track finished normally) or any other negative
                // code (e.g. AEL_IO_FAIL) -- either way there's nothing more
                // to play from this track, so auto-advance to the next one.
                player_advance_and_play(next_index(s_player.track_index));
            }
        }
    }
}

/* ============================== I2S / GPIO setup ============================== */

static void i2s_mic_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create microphone I2S channel (%s) -- check microphone "
                      "wiring (WS/SCK/SD to D1/D2/D3)", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        // INMP441 needs 32-bit slot width even though it only outputs 24 bits of data
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_SCK_IO,
            .ws   = MIC_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_SD_IO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT; // L/R tied to GND -> left channel

    err = i2s_channel_init_std_mode(s_rx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure microphone I2S mode (%s) -- check microphone "
                      "wiring (WS/SCK/SD to D1/D2/D3)", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);

    err = i2s_channel_enable(s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable microphone I2S channel (%s) -- check microphone "
                      "wiring (WS/SCK/SD to D1/D2/D3)", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);
}

static void i2s_speaker_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_handle, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create amplifier I2S channel (%s) -- check amplifier "
                      "wiring (BCLK/WS/DOUT to D5/D4/D6)", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(PLAYBACK_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPK_BCLK_IO,
            .ws   = SPK_WS_IO,
            .dout = SPK_DOUT_IO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    // Mono mode transmits the same sample on both wire slots automatically.
    // Since both slots are identical, the MAX98357A's default "(L+R)/2"
    // stereo-average output reproduces the original signal exactly -- no
    // GAIN/SD_MODE resistor is needed.
    err = i2s_channel_init_std_mode(s_tx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure amplifier I2S mode (%s) -- check amplifier "
                      "wiring (BCLK/WS/DOUT to D5/D4/D6)", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);

    err = i2s_channel_enable(s_tx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable amplifier I2S channel (%s) -- check amplifier "
                      "wiring (BCLK/WS/DOUT to D5/D4/D6)", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(err);
}

static void led_init(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // off (active-low)
}
static inline void led_on(void)  { gpio_set_level(LED_GPIO, 0); }
static inline void led_off(void) { gpio_set_level(LED_GPIO, 1); }

// External LED strip on D0, faded with the LEDC hardware PWM peripheral.
// Fades run in hardware once started, so fade_in()/fade_out() return
// immediately and never block the caller.
static void strip_led_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = STRIP_LED_PWM_MODE,
        .timer_num       = STRIP_LED_PWM_TIMER,
        .duty_resolution = STRIP_LED_PWM_RES,
        .freq_hz         = STRIP_LED_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .gpio_num   = STRIP_LED_GPIO,
        .speed_mode = STRIP_LED_PWM_MODE,
        .channel    = STRIP_LED_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = STRIP_LED_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

static inline void strip_led_fade_in(void) {
    ledc_set_fade_with_time(STRIP_LED_PWM_MODE, STRIP_LED_PWM_CHANNEL, STRIP_LED_PWM_MAX_DUTY, LED_FADE_TIME);
    ledc_fade_start(STRIP_LED_PWM_MODE, STRIP_LED_PWM_CHANNEL, LEDC_FADE_NO_WAIT);
}

static inline void strip_led_fade_out(void) {
    ledc_set_fade_with_time(STRIP_LED_PWM_MODE, STRIP_LED_PWM_CHANNEL, 0, LED_FADE_TIME);
    ledc_fade_start(STRIP_LED_PWM_MODE, STRIP_LED_PWM_CHANNEL, LEDC_FADE_NO_WAIT);
}

static void mic_read_chunk(int16_t *out, int chunksize) {
    size_t bytes_read = 0;
    int32_t raw[chunksize];
    i2s_channel_read(s_rx_handle, raw, chunksize * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    int samples = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples; i++) {
        out[i] = (int16_t)(raw[i] >> 14); // scale 32-bit I2S sample down to 16-bit
    }
}

/* ============================== SD card mount ============================== */

static esp_err_t sd_card_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_IO,
        .miso_io_num = SD_MISO_IO,
        .sclk_io_num = SD_SCK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus for SD card (%s) -- check SD card "
                      "wiring (SCK/MISO/MOSI to D8/D7/D9)", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_IO;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem -- card present but unreadable. "
                          "Use a 2GB or smaller SD card: larger cards are often "
                          "formatted in a way this mounter can't read, and format "
                          "issues are the most common cause of this error.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card (%s) -- check SD card wiring "
                          "(SCK/MISO/MOSI/CS to D8/D7/D9/D10)", esp_err_to_name(ret));
        }
        return ret;
    }
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

/* ============================== Voice task ============================== */

static inline void send_cmd(player_cmd_t c) {
    xQueueSend(s_cmd_queue, &c, 0);
}

static const char *wake_word_for_id(int id) {
    switch (id) {
    case CMD_WAKE1_ID: return WAKE_WORD_1;
    case CMD_WAKE2_ID: return WAKE_WORD_2;
    default:           return WAKE_WORD_3;
    }
}

typedef enum { VOICE_IDLE, VOICE_LISTENING } voice_state_t;

static void voice_task(void *arg) {
    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == NULL) {
        ESP_LOGE(TAG, "Failed to load models from 'model' partition");
        vTaskDelete(NULL);
        return;
    }

    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (mn_name == NULL) {
        ESP_LOGE(TAG, "No English MultiNet model found - check menuconfig MultiNet selection");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Using MultiNet model: %s", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *mn_data = multinet->create(mn_name, MN_CREATE_DURATION_MS);
    int mn_chunksize = multinet->get_samp_chunksize(mn_data);

    esp_mn_commands_clear();
    esp_mn_commands_add(CMD_WAKE1_ID,    WAKE_WORD_1);
    esp_mn_commands_add(CMD_WAKE2_ID,    WAKE_WORD_2);
    esp_mn_commands_add(CMD_WAKE3_ID,    WAKE_WORD_3);
    esp_mn_commands_add(CMD_PLAY_ID,     "PLAY");
    esp_mn_commands_add(CMD_STOP_ID,     "STOP");
    esp_mn_commands_add(CMD_BACK_ID,     "BACK");
    esp_mn_commands_add(CMD_NEXT_ID,     "NEXT");
    esp_mn_commands_add(CMD_RESTART_ID,  "RESTART");
    esp_mn_commands_add(CMD_VOL_DOWN_ID, "VOLUME DOWN");
    esp_mn_commands_add(CMD_VOL_UP_ID,   "VOLUME UP");
    esp_mn_commands_update();
    multinet->print_active_speech_commands(mn_data);

    int16_t *buffer = malloc(mn_chunksize * sizeof(int16_t));
    assert(buffer);

    voice_state_t state = VOICE_IDLE;
    int64_t listen_deadline_us = 0;

    ESP_LOGI(TAG, "Listening continuously for '%s', '%s', or '%s'...",
             WAKE_WORD_1, WAKE_WORD_2, WAKE_WORD_3);

    while (1) {
        mic_read_chunk(buffer, mn_chunksize);
        esp_mn_state_t mn_state = multinet->detect(mn_data, buffer);

        if (mn_state == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *res = multinet->get_results(mn_data);
            if (res->num > 0) {
                int id = res->command_id[0];

                if (state == VOICE_IDLE) {
                    if (id == CMD_WAKE1_ID || id == CMD_WAKE2_ID || id == CMD_WAKE3_ID) {
                        printf("Wake word detected: %s\n", wake_word_for_id(id));
                        led_on();
                        strip_led_fade_in();
                        send_cmd(CMD_DUCK_VOLUME); // duck music to LISTENING_VOLUME while we listen
                        state = VOICE_LISTENING;
                        listen_deadline_us = esp_timer_get_time() + LISTEN_WINDOW_US;
                    }
                    // Any other phrase heard while idle is ignored -- a
                    // command only counts inside the post-wake-word window.
                } else { // VOICE_LISTENING
                    switch (id) {
                    case CMD_WAKE1_ID:
                    case CMD_WAKE2_ID:
                    case CMD_WAKE3_ID:
                        // Heard again mid-window: just refresh the timer.
                        // Already ducked and faded in, so don't repeat those.
                        listen_deadline_us = esp_timer_get_time() + LISTEN_WINDOW_US;
                        break;
                    case CMD_PLAY_ID:
                        printf("Command: PLAY\n");
                        send_cmd(CMD_RESTORE_VOLUME); // back to normal volume before acting
                        send_cmd(CMD_PLAY);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    case CMD_STOP_ID:
                        printf("Command: STOP\n");
                        send_cmd(CMD_RESTORE_VOLUME);
                        send_cmd(CMD_STOP);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    case CMD_BACK_ID:
                        printf("Command: BACK\n");
                        send_cmd(CMD_RESTORE_VOLUME);
                        send_cmd(CMD_BACK);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    case CMD_NEXT_ID:
                        printf("Command: NEXT\n");
                        send_cmd(CMD_RESTORE_VOLUME);
                        send_cmd(CMD_NEXT);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    case CMD_RESTART_ID:
                        printf("Command: RESTART\n");
                        send_cmd(CMD_RESTORE_VOLUME);
                        send_cmd(CMD_RESTART);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    case CMD_VOL_DOWN_ID:
                        printf("Command: VOLUME DOWN\n");
                        send_cmd(CMD_RESTORE_VOLUME); // restore first, then step down from THAT volume
                        send_cmd(CMD_VOL_DOWN);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    case CMD_VOL_UP_ID:
                        printf("Command: VOLUME UP\n");
                        send_cmd(CMD_RESTORE_VOLUME); // restore first, then step up from THAT volume
                        send_cmd(CMD_VOL_UP);
                        led_off(); strip_led_fade_out(); state = VOICE_IDLE;
                        break;
                    default:
                        break;
                    }
                }
            }
            multinet->clean(mn_data); // reset internal audio history, keep listening
        } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
            multinet->clean(mn_data);
        }

        // Our own UI timeout is independent of MultiNet's internal one
        // (MN_CREATE_DURATION_MS), and is what actually governs the LEDs.
        if (state == VOICE_LISTENING && esp_timer_get_time() >= listen_deadline_us) {
            led_off();
            strip_led_fade_out();
            state = VOICE_IDLE;
            send_cmd(CMD_RESTORE_VOLUME); // no command came in time -- put the volume back anyway
            multinet->clean(mn_data);
        }
    }

    multinet->destroy(mn_data);
    free(buffer);
}

/* ============================== app_main ============================== */

void app_main(void) {
    led_init();
    strip_led_init();

    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "Continuing without a usable SD card -- voice commands "
                      "will be recognized, but playback will have no tracks to play "
                      "until the card is fixed and the board is reset.");
    }
    scan_tracks();

    i2s_mic_init();
    i2s_speaker_init();
    player_pipeline_init();

    s_cmd_queue = xQueueCreate(8, sizeof(player_cmd_t));
    assert(s_cmd_queue);

    // Pinned to separate cores -- see PLAYBACK_CORE/VOICE_CORE above.
    xTaskCreatePinnedToCore(playback_task, "playback", 6144, NULL, 5, NULL, PLAYBACK_CORE);
    xTaskCreatePinnedToCore(voice_task,    "voice",    8192, NULL, 5, NULL, VOICE_CORE);
}