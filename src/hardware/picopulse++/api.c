/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2022 Shawn Walker <ac0bi00@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <config.h>
#include <fcntl.h>
#include <glib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

/* Baud rate is really a don't care because we run USB CDC, dtr must be 1.
 * flow should be zero since we don't use xon/xoff */
#define SERIALCOMM "115200/8n1/dtr=1/rts=0/flow=0"

/* Use the force_detect scan option as a way to pass user information to the
 * device the string must use only 0-9,a-z,A-Z,'.','=' and '-'* and be less than
 * 60 characters */

static const uint32_t scanopts[] = {
    SR_CONF_CONN, /* Required OS name for the port, i.e. /dev/ttyACM0 */
    SR_CONF_SERIALCOMM, /* Optional config of the port, i.e. 115200/8n1 */
    SR_CONF_FORCE_DETECT
};

/* Sample rate can either provide a std_gvar_samplerates_steps or a
 * std_gvar_samplerates. The latter is just a long list of every supported rate.
 * For the steps, pulseview/pv/toolbars/mainbar.cpp will do a min,max,step. If
 * step is 1 then it provides a 1,2,5,10 select otherwise it allows a spin box.
 * Going with the full list because while the spin box is more flexible, it is
 * harder to read */
static const uint64_t samplerates[] = {
    /* --- Streaming-Bereich (5 kHz bis 15 MHz) --- */
    SR_KHZ(5),      /* 200 MHz / 40000 */
    SR_KHZ(10),     /* 200 MHz / 20000 */
    SR_KHZ(20),     /* 200 MHz / 10000 */
    SR_KHZ(25),     /* 200 MHz / 8000  */
    SR_KHZ(40),     /* 200 MHz / 5000  */
    SR_KHZ(50),     /* 200 MHz / 4000  (Max bei 4 ADC-Kanälen) */
    SR_HZ(66666),   /* 200 kHz / 3     (Max bei 3 ADC-Kanälen: 66,67 kHz) */
    SR_KHZ(80),     /* 200 MHz / 2500  */
    SR_KHZ(100),    /* 200 MHz / 2000  (Max bei 2 ADC-Kanälen) */
    SR_KHZ(125),    /* 200 MHz / 1600  */
    SR_KHZ(200),    /* 200 MHz / 1000  (Max bei 1 ADC-Kanal) */
    SR_KHZ(250),    /* 200 MHz / 800   */
    SR_KHZ(400),    /* 200 MHz / 500   */
    SR_KHZ(500),    /* 200 MHz / 400   */
    SR_KHZ(800),    /* 200 MHz / 250   */
    SR_MHZ(1),      /* 200 MHz / 200   */
    SR_MHZ(2),      /* 200 MHz / 100   */
    SR_MHZ(4),      /* 200 MHz / 50    */
    SR_MHZ(5),      /* 200 MHz / 40    */
    SR_MHZ(8),      /* 200 MHz / 25    */
    SR_MHZ(10),     /* 200 MHz / 20    */
    SR_KHZ(12500),  /* 200 MHz / 16    */
    SR_MHZ(15),     /* 200 MHz / 13.33 (max. Streaming-Rate) */

    /* --- High-Speed Burst-Bereich (> 15 MHz bis 200 MHz) --- */
    SR_MHZ(20),     /* 200 MHz / 10    */
    SR_MHZ(25),     /* 200 MHz / 8     */
    SR_MHZ(40),     /* 200 MHz / 5     */
    SR_MHZ(50),     /* 200 MHz / 4     */
    SR_MHZ(100),    /* 200 MHz / 2     */
    SR_MHZ(200),    /* 200 MHz / 1     */
};

static const uint32_t drvopts[] = {
    SR_CONF_OSCILLOSCOPE,
    SR_CONF_LOGIC_ANALYZER,
};

static const int32_t trigger_matches[] = {
    SR_TRIGGER_ZERO,
    SR_TRIGGER_ONE,
    SR_TRIGGER_RISING,
    SR_TRIGGER_FALLING,
    SR_TRIGGER_EDGE,
};

static const uint32_t devopts[] = {
    SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
    SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
    SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static struct sr_dev_driver picopulse_driver_info;


static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
    struct sr_config *src;
    struct sr_dev_inst *sdi;
    struct sr_serial_dev_inst *serial;
    struct dev_context *devc;
    struct sr_channel *ch;
    GSList *l;
    int num_read;
    int i;
    const char *conn, *serialcomm, *force_detect;
    char buf[32];
    char ustr[64];
    int len;
    uint8_t num_a, num_d, a_size;
    gchar *channel_name;
    char name_cmd[20];

    conn = serialcomm = force_detect = NULL;
    for (l = options; l; l = l->next) {
        src = l->data;
        switch (src->key) {
            case SR_CONF_CONN:
                conn = g_variant_get_string(src->data, NULL);
                break;
            case SR_CONF_SERIALCOMM:
                serialcomm = g_variant_get_string(src->data, NULL);
                break;
            case SR_CONF_FORCE_DETECT:
                force_detect = g_variant_get_string(src->data, NULL);
                sr_info("Force detect string %s", force_detect);
                break;
        }
    }
    if (!conn)
        return NULL;

    if (!serialcomm)
        serialcomm = SERIALCOMM;

    serial = sr_serial_dev_inst_new(conn, serialcomm);
    sr_info("Opening %s.", conn);
    if (serial_open(serial, SERIAL_RDWR) != SR_OK) {
        sr_err("1st serial open fail");
        return NULL;
    }

    sr_info("Resetting device with *s at %s.", conn);
    send_serial_char_pp(serial, '*');
    g_usleep(10000);
    do {
        sr_warn("Drain reads");
        len = serial_read_blocking(serial, buf, 32, 100);
        sr_warn("Drain reads done");
        if (len)
            sr_dbg("Dropping in flight serial data");
    } while (len > 0);
    /* Send the user string with the identify */
    if (force_detect && (strlen(force_detect) <= 60)) {
        sprintf(ustr, "i%s\n", force_detect);
        sr_info("User string %s", ustr);
        num_read = send_serial_w_resp_pp(serial, ustr, buf, 17);
    } else {
        num_read = send_serial_w_resp_pp(serial, "i\n", buf, 17);
    }
    if (num_read < 16) {
        sr_err("1st identify failed");
        serial_close(serial);
        g_usleep(100000);
        if (serial_open(serial, SERIAL_RDWR) != SR_OK) {
            sr_err("2nd serial open fail");
            return NULL;
        }
        g_usleep(100000);
        sr_err("Send second *");
        send_serial_char_pp(serial, '*');
        g_usleep(100000);
        num_read = send_serial_w_resp_pp(serial, "i\n", buf, 17);
        if (num_read < 10) {
            sr_err("Second attempt failed");
            return NULL;
        }
    }

    /* Expected ID response is SRPICO,AxxyDzz,VV
     * where xx are number of analog channels, y is bytes per analog sample
     * (7 bits per byte), zz is number of digital channels, and VV is two digit
     * version# which must be 02 */
    if ((num_read < 16) || (strncmp(buf, "SRPICO,A", 8))
        || (buf[11] != 'D') || (buf[15] != '0') || (buf[16] != '2')) {
        sr_err("ERROR: Bad response string %s %d", buf, num_read);
        return NULL;
    }

    a_size = buf[10] - '0';
    buf[10] = '\0'; /*Null to end the str for atois */
    buf[14] = '\0';
    num_a = atoi(&buf[8]);
    num_d = atoi(&buf[12]);

    sdi = g_malloc0(sizeof(struct sr_dev_inst));
    sdi->status = SR_ST_INACTIVE;
    sdi->vendor = g_strdup("FH Aachen");
    sdi->model = g_strdup("picoPULSE++");
    sdi->version = g_strdup("00");
    sdi->conn = serial;
    sdi->driver = &picopulse_driver_info;
    sdi->inst_type = SR_INST_SERIAL;
    sdi->serial_num = g_strdup("N/A");

    if (((num_a == 0) && (num_d == 0))
        || (num_a > MAX_ANALOG_CHANNELS) || (num_d > MAX_DIGITAL_CHANNELS)
        || (a_size < 1) || (a_size > 4)) {
        sr_err("ERROR: invalid channel config a %d d %d asz %d",
               num_a, num_d, a_size);
        return NULL;
    }

    devc = g_malloc0(sizeof(struct dev_context));
    devc->a_size = a_size;
    devc->num_a_channels = num_a;
    devc->num_d_channels = num_d;
    devc->a_chan_mask = ((1 << num_a) - 1);
    devc->d_chan_mask = ((1 << num_d) - 1);

    /* The number of bytes that each digital sample in the buffers sent to the
     * session. All logical channels are packed together, where a slice of N
     * channels takes roundup(N/8) bytes. This never changes even if channels
     * are disabled because PV expects disabled channels to still be accounted
     * for in the packing */
    devc->dig_sample_bytes = ((devc->num_d_channels + 7) / 8);
    /* These are the slice sizes of the data on the wire
     * 1 7 bit field per byte */
    devc->bytes_per_slice = (devc->num_a_channels * devc->a_size);

    if (devc->num_d_channels > 0) {
        /* logic sent in groups of 7*/
        devc->bytes_per_slice += (devc->num_d_channels + 6) / 7;
    }
    sr_dbg("num channels a %d d %d bps %d dsb %d", num_a, num_d,
           devc->bytes_per_slice, devc->dig_sample_bytes);

    /* Each analog channel is its own group; digital are just channels;
     * Grouping of channels is rather arbitrary as parameters like sample rate
     * and number of samples apply to all channels. Analog channels do have a
     * scale and offset, but that is applied automatically. */
    devc->analog_groups = g_malloc0(sizeof(struct sr_channel_group *) *
                                    devc->num_a_channels);

    /* Dynamische Analog-Kanalnamen vom Device abfragen */
    for (i = 0; i < devc->num_a_channels; i++) {
        sprintf(name_cmd, "nA%d\n", i);
        num_read = send_serial_w_resp_pp(serial, name_cmd, buf, 17);
        if (num_read > 0) {
            buf[num_read] = 0;
            g_strstrip(buf);
            channel_name = g_strdup(buf);
        } else {
            channel_name = g_strdup_printf("A%d", i);
        }
        ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG, TRUE, channel_name);
        devc->analog_groups[i] = g_malloc0(sizeof(struct sr_channel_group));
        devc->analog_groups[i]->name = channel_name;
        devc->analog_groups[i]->channels = g_slist_append(NULL, ch);
        sdi->channel_groups = g_slist_append(sdi->channel_groups,
                                             devc->analog_groups[i]);
    }

    /* Dynamische Digital-Kanalnamen vom Device abfragen */
    if (devc->num_d_channels > 0) {
        for (i = 0; i < devc->num_d_channels; i++) {
            sprintf(name_cmd, "nD%d\n", i);
            num_read = send_serial_w_resp_pp(serial, name_cmd, buf, 17);
            if (num_read > 0) {
                buf[num_read] = 0;
                g_strstrip(buf);
                channel_name = g_strdup(buf);
            } else {
                channel_name = g_strdup_printf("D%d", i + 2);
            }
            sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, channel_name);
            g_free(channel_name);
        }
    }

    /* In large sample usages we get the call to receive with large transfers.
     * Since the CDC serial implemenation can silenty lose data as it gets close
     * to full, allocate storage for a half buffer which in a worst case
     * scenario has 2x ratio of transmitted bytes to storage bytes.
     * Note: The intent of making this buffer large is to prevent CDC serial
     * buffer overflows. However, it is likely that if the host is running slow
     * (i.e. it's a raspberry pi model 3) that it becomes compute bound and
     * doesn't service CDC serial responses in time to not overflow the internal
     * CDC buffers.	Thus no serial buffer is large enough.
     * But, it's only 32K... */
    devc->serial_buffer_size = 32000;
    devc->buffer = NULL;
    sr_dbg("Setting serial buffer size: %i.", devc->serial_buffer_size);

    devc->cbuf_wrptr = 0;
    /* While slices are sent as a group of one sample across all channels,
     * sigrok wants analog channel data sent as separate packets. Logical trace
     * values are packed together. An RLE byte in normal mode can represent up
     * to 1640 samples. In D4 an RLE byte can represent up to 640 samples.
     * Rather than making the sample_buf_size 1640x the size of serial buffer,
     * we require that the process loops push samples to the session as we get
     * anywhere close to full. */

    devc->sample_buf_size = devc->serial_buffer_size;
    for (i = 0; i < devc->num_a_channels; i++) {
        devc->a_data_bufs[i] = NULL;
        devc->a_pretrig_bufs[i] = NULL;
    }
    devc->d_data_buf = NULL;
    devc->sample_rate = 5000;
    devc->capture_ratio = 10;
    devc->rxstate = RX_IDLE;
    /*Set an initial value as various code relies on an inital value. */
    devc->limit_samples = 1000;
    /* NEU: Initialwerte für dynamische RP2040-Limits */
    devc->last_queried_a_mask = 0xFFFFFFFF;
    devc->last_queried_d_mask = 0xFFFFFFFF;
    devc->min_samplerate = 5000;
    devc->max_stream_rate = 15000000;
    devc->max_bulk_rate = 200000000;
    devc->max_bulk_samples = 100000;

    sdi->priv = devc;

    if (picopulse_get_dev_cfg(sdi) != SR_OK) {
        return NULL;
    };

    serial_close(serial);
    return std_scan_complete(di, g_slist_append(NULL, sdi));
}

/* Note that on the initial driver load we pull all values into local storage.
 * Thus gets can return local data, but sets have to issue commands to device. */
static int config_set(uint32_t key, GVariant *data,
        const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
    struct dev_context *devc;
    int ret;
    (void) cg;

    if (!sdi)
        return SR_ERR_ARG;

    devc = sdi->priv;
    ret = SR_OK;

    sr_dbg("Got config_set key %d \n", key);
    switch (key) {
        case SR_CONF_SAMPLERATE:
            devc->sample_rate = g_variant_get_uint64(data);
            if (devc->sample_rate > devc->max_bulk_rate)
                devc->sample_rate = devc->max_bulk_rate;
            if (devc->sample_rate < devc->min_samplerate)
                devc->sample_rate = devc->min_samplerate;
            sr_dbg("config_set sr %lu\n", devc->sample_rate);
            break;
        case SR_CONF_LIMIT_SAMPLES:
            devc->limit_samples = g_variant_get_uint64(data);
            if (devc->sample_rate > devc->max_stream_rate &&
                devc->limit_samples > devc->max_bulk_samples)
                devc->limit_samples = devc->max_bulk_samples;
            sr_dbg("config_set slimit %" PRIu64 "\n", devc->limit_samples);
            break;
        case SR_CONF_CAPTURE_RATIO:
            devc->capture_ratio = g_variant_get_uint64(data);
            break;

        default:
            sr_err("ERROR: config_set given undefined key %d\n", key);
            ret = SR_ERR_NA;
    }

    return ret;
}

static int config_get(uint32_t key, GVariant **data,
        const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
    struct dev_context *devc;

    sr_dbg("config_get given key %d", key);

    (void) cg;

    if (!sdi)
        return SR_ERR_ARG;

    devc = sdi->priv;
    switch (key) {
        case SR_CONF_SAMPLERATE:
            *data = g_variant_new_uint64(devc->sample_rate);
            sr_spew("sample rate get of %" PRIu64 "", devc->sample_rate);
            break;
        case SR_CONF_CAPTURE_RATIO:
            if (!sdi)
                return SR_ERR;
            devc = sdi->priv;
            *data = g_variant_new_uint64(devc->capture_ratio);
            break;
        case SR_CONF_LIMIT_SAMPLES:
            sr_spew("config_get limit_samples of %lu", devc->limit_samples);
            *data = g_variant_new_uint64(devc->limit_samples);
            break;
        default:
            sr_spew("unsupported config_get key %d", key);
            return SR_ERR_NA;
    }
    return SR_OK;
}

static void picopulse_sync_channel_limits(const struct sr_dev_inst *sdi) {
    struct dev_context *devc = sdi->priv;
    struct sr_serial_dev_inst *serial = sdi->conn;
    uint32_t cur_a_mask = 0, cur_d_mask = 0;
    char cmd[32], resp[64];
    int num_read;

    /* Aktuelle Kanal-Bitmasken aus PulseView-Konfiguration bauen */
    for (GSList *l = sdi->channels; l; l = l->next) {
        struct sr_channel *ch = l->data;
        if (!ch->enabled)
            continue;
        if (ch->type == SR_CHANNEL_ANALOG)
            cur_a_mask |= (1U << ch->index);
        else if (ch->type == SR_CHANNEL_LOGIC)
            cur_d_mask |= (1U << ch->index);
    }

    /* Nur abfragen, wenn sich die Maske geändert hat und der Port offen ist */
    if ((cur_a_mask == devc->last_queried_a_mask) &&
        (cur_d_mask == devc->last_queried_d_mask))
        return;

    if (!serial) return;

    snprintf(cmd, sizeof(cmd), "M%X,%X\n", cur_a_mask, cur_d_mask);
    sr_warn("TX M-cmd: %s", cmd);
    num_read = send_serial_w_resp_pp(serial, cmd, resp, sizeof(resp) - 1);
    sr_warn("RX M-resp (%d): %s", num_read, num_read > 0 ? resp : "<empty>");

    if (num_read > 1 && resp[0] == '*') {
        resp[num_read] = '\0';
        uint64_t min_r, max_stream, max_bulk, max_samples;
        if (sscanf(&resp[1], "%" SCNu64 ",%" SCNu64 ",%" SCNu64 ",%" SCNu64,
                   &min_r, &max_stream, &max_bulk, &max_samples) == 4) {
            devc->min_samplerate = min_r;
            devc->max_stream_rate = max_stream;
            devc->max_bulk_rate = max_bulk;
            devc->max_bulk_samples = max_samples;
            devc->last_queried_a_mask = cur_a_mask;
            devc->last_queried_d_mask = cur_d_mask;

            /* Geisterwerte sofort auf gültige Grenzen klemmen */
            if (devc->sample_rate > devc->max_bulk_rate)
                devc->sample_rate = devc->max_bulk_rate;
            if (devc->sample_rate < devc->min_samplerate)
                devc->sample_rate = devc->min_samplerate;
            if (devc->sample_rate > devc->max_stream_rate &&
                devc->limit_samples > devc->max_bulk_samples)
                devc->limit_samples = devc->max_bulk_samples;
        }
    }
}

static int config_list(uint32_t key, GVariant **data,
        const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
    struct dev_context *devc;

    if (cg != NULL) {
        return SR_ERR_NA;
    }

    if ((key == SR_CONF_SCAN_OPTIONS) || (key == SR_CONF_DEVICE_OPTIONS)) {
        return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
    }

    if (!sdi) {
        sr_err("ERROR: Call to config list with null sdi");
        return SR_ERR_ARG;
    }

    devc = sdi->priv;

    /* Bei jeder Abfrage von Rate oder Samples zuerst mit RP2040 synchronisieren */
    if (key == SR_CONF_SAMPLERATE || key == SR_CONF_LIMIT_SAMPLES) {
        picopulse_sync_channel_limits(sdi);
    }

    switch (key) {
        case SR_CONF_SAMPLERATE: {
            uint64_t valid_rates[ARRAY_SIZE(samplerates)];
            size_t count = 0;

            for (size_t i = 0; i < ARRAY_SIZE(samplerates); i++) {
                if (samplerates[i] >= devc->min_samplerate &&
                    samplerates[i] <= devc->max_bulk_rate) {
                    valid_rates[count++] = samplerates[i];
                    }
            }
            if (count == 0) {
                valid_rates[0] = devc->min_samplerate ? devc->min_samplerate : 5000;
                count = 1;
            }
            *data = std_gvar_samplerates(valid_rates, count);
            break;
        }
        case SR_CONF_TRIGGER_MATCH:
            *data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
            break;
        case SR_CONF_LIMIT_SAMPLES: {
            uint64_t max_lim = (devc->sample_rate <= devc->max_stream_rate)
                ? 1000000000ULL : devc->max_bulk_samples;
            if (max_lim < 1)
                max_lim = 1;
            *data = std_gvar_tuple_u64(1ULL, max_lim);
            break;
        }
        default:
            sr_dbg("Reached default statement of config_list");

            return SR_ERR_NA;
    }

    return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
    struct sr_serial_dev_inst *serial;
    struct dev_context *devc;
    struct sr_channel *ch;
    struct sr_trigger *trigger;
    char tmpstr[32];
    char buf[32];
    GSList *l;
    int a_enabled = 0, d_enabled = 0, len;
    serial = sdi->conn;
    int i, num_read;

    devc = sdi->priv;
    sr_dbg("Enter acq start");
    sr_dbg("dsbstart %d", devc->dig_sample_bytes);

    devc->buffer = g_malloc(devc->serial_buffer_size);
    if (!(devc->buffer)) {
        sr_err("ERROR: serial buffer malloc fail");
        return SR_ERR_MALLOC;
    }

    /* Get device in idle state and flush any delayed USB CDC packets */
    send_serial_char_pp(serial, '*');
    g_usleep(10000);
    do {
        len = serial_read_blocking(serial, devc->buffer,
                                   devc->serial_buffer_size, 20);
        if (len > 0)
            sr_dbg("Draining %d stale bytes before start", len);
    } while (len > 0);

    memset(devc->d_last, 0, sizeof(devc->d_last));

    for (l = sdi->channels; l; l = l->next) {
        ch = l->data;
        sr_dbg("c %d enabled %d name %s\n", ch->index, ch->enabled, ch->name);

        if (ch->type == SR_CHANNEL_ANALOG) {
            devc->a_chan_mask &= ~(1 << ch->index);
            if (ch->enabled) {
                devc->a_chan_mask |= (ch->enabled << ch->index);
                a_enabled++;
            }
            sprintf(tmpstr, "A%d%d\n", ch->enabled, ch->index);
        } else if (ch->type == SR_CHANNEL_LOGIC) {
            devc->d_chan_mask &= ~(1 << ch->index);
            if (ch->enabled) {
                devc->d_chan_mask |= (ch->enabled << ch->index);
                d_enabled++;
            }
            sprintf(tmpstr, "D%d%d\n", ch->enabled, ch->index);
        } else {
            sr_err("ERROR: Channel enable encountered unknown type %d", ch->type);
            return SR_ERR;
        }

        sr_info("Channel enable masks D 0x%X A 0x%X",
                devc->d_chan_mask, devc->a_chan_mask);
        if (send_serial_w_ack_pp(serial, tmpstr) != SR_OK) {
            sr_err("ERROR: Channel enable fail");
            return SR_ERR;
        }
    }

    /* Recalculate bytes_per_slice based on which analog channels are enabled */
    devc->bytes_per_slice = (a_enabled * devc->a_size);

    for (i = 0; i < devc->num_d_channels; i += 7)
        if (((devc->d_chan_mask) >> i) & (0x7F))
            (devc->bytes_per_slice)++;

    if ((a_enabled == 0) && (d_enabled == 0)) {
        sr_err("ERROR:No channels enabled");
        return SR_ERR;
    }

    sr_dbg("bps %d\n", devc->bytes_per_slice);

    /* Depending on channel configs, rates below 5ksps are possible but such a
     * low rate can easily stream and this eliminates a lot	of special cases. */
    if (devc->sample_rate < devc->min_samplerate) {
        sr_err("Sample rate override to min of %" PRIu64, devc->min_samplerate);
        devc->sample_rate = devc->min_samplerate;
    }

    /* While PICO specs a max clock ~120-125Mhz, it does overclock in many cases
     * so leaving a warning. */
    if (devc->sample_rate > devc->max_bulk_rate)
        sr_warn("WARN: Sample rate above max bulk rate %" PRIu64, devc->max_bulk_rate);


    /* To support future devices that may allow the analog scale/offset to
     * chang e, call get_dev_cfg again to get new values */
    if (picopulse_get_dev_cfg(sdi) != SR_OK) {
        sr_err("get_dev_cfg failure on start");
        return SR_ERR;
    }

    /* With all other params set, we use the final sample rate setting as an
     * opportunity for the device to communicate any errors or rate clamping.
     * Reply format: "*" or "*<effective_sample_rate>" */
    sprintf(tmpstr, "R%lu\n", devc->sample_rate);
    num_read = send_serial_w_resp_pp(serial, tmpstr, buf, 30);
    buf[num_read] = 0;
    if ((num_read > 1) && (buf[0] == '*')) {
        uint64_t eff_rate = strtoull(&buf[1], NULL, 10);
        if (eff_rate > 0 && eff_rate != devc->sample_rate) {
            sr_info("Device adjusted sample rate from %" PRIu64 " Hz to %" PRIu64 " Hz",
                devc->sample_rate, eff_rate);
            devc->sample_rate = eff_rate;
        }
        sr_dbg("Sample rate to device success with resp %s", buf);
    } else if (!((num_read == 1) && (buf[0] == '*'))) {
        sr_err("Sample rate to device failed");
        if (num_read > 0) {
            buf[num_read] = 0;
            sr_err("sample_rate error string %s", buf);
        }
        return SR_ERR;
    }

    devc->sent_samples = 0;
    devc->byte_cnt = 0;
    devc->bytes_avail = 0;
    devc->wrptr = 0;
    devc->cbuf_wrptr = 0;
    len = serial_read_blocking(serial, devc->buffer, devc->serial_buffer_size,
                               serial_timeout(serial, 4));

    if (len > 0) {
        sr_info("Pre-ARM drain had %d characters:", len);
        devc->buffer[len] = 0;
        sr_info("%s", devc->buffer);
    }

    for (i = 0; i < devc->num_a_channels; i++) {
        devc->a_data_bufs[i] = g_malloc(devc->sample_buf_size * sizeof(float));
        if (!(devc->a_data_bufs[i])) {
            sr_err("ERROR: analog buffer malloc fail");
            return SR_ERR_MALLOC;
        }
    }

    if (devc->num_d_channels > 0) {
        devc->d_data_buf = g_malloc(devc->sample_buf_size *
                                    devc->dig_sample_bytes);
        if (!(devc->d_data_buf)) {
            sr_err("ERROR: logic buffer malloc fail");
            return SR_ERR_MALLOC;
        }
    }

    devc->pretrig_entries = (devc->capture_ratio * devc->limit_samples) / 100;

    if ((trigger = sr_session_trigger_get(sdi->session))) {
        if (devc->sample_rate <= devc->max_stream_rate) {
            /* 1. Streaming-Bereich (<= 15 MHz): Soft-Trigger auf dem PC */
            if (send_serial_w_ack_pp(serial, "L4294967295\n") != SR_OK) {
                sr_err("Continuous sample limit to device failed");
                return SR_ERR;
            }

            devc->stl = soft_trigger_logic_new(sdi, trigger, devc->pretrig_entries);
            if (!devc->stl)
                return SR_ERR_MALLOC;

            /* Analoge Pre-Trigger-Ringpuffer anlegen */
            devc->pretrig_wr_ptr = 0;
            if (devc->pretrig_entries > 0) {
                for (i = 0; i < devc->num_a_channels; i++) {
                    if ((devc->a_chan_mask >> i) & 1) {
                        devc->a_pretrig_bufs[i] = g_malloc0(
                            devc->pretrig_entries * sizeof(float));
                        if (!devc->a_pretrig_bufs[i]) {
                            sr_err("ERROR: analog pretrig buffer malloc fail");
                            return SR_ERR_MALLOC;
                        }
                    }
                }
            }

            devc->trigger_fired = FALSE;

            sr_info("Entering sw triggered mode (continuous stream)");

            serial_source_add(sdi->session, serial, G_IO_IN, 200,
                              picopulse_receive, (void *) sdi);

            if (send_serial_str_pp(serial, "F\n") != SR_OK)
                return SR_ERR;
        } else {
            /* 2. High-Speed Burst-Bereich (> 15 MHz): Hardware-Trigger auf RP2040 */
            struct sr_trigger_stage *stage;
            struct sr_trigger_match *match;
            GSList *m;

            devc->stl = NULL;
            devc->trigger_fired = FALSE;

            sprintf(tmpstr, "L%" PRIu64 "\n", devc->limit_samples);
            if (send_serial_w_ack_pp(serial, tmpstr) != SR_OK) {
                sr_err("Sample limit to device failed");
                return SR_ERR;
            }

            if (g_slist_length(trigger->stages) > 1)
                return SR_ERR_NA;

            stage = trigger->stages->data;
            if (!stage)
                return SR_ERR_ARG;

            for (m = stage->matches; m; m = m->next) {
                match = m->data;
                if (!match->channel->enabled)
                    continue;
                int idx = match->channel->index;
                int val;
                switch (match->match) {
                    case SR_TRIGGER_ZERO:
                        val = 0; break;
                    case SR_TRIGGER_ONE:
                        val = 1; break;
                    case SR_TRIGGER_RISING:
                        val = 2; break;
                    case SR_TRIGGER_FALLING:
                        val = 3; break;
                    case SR_TRIGGER_EDGE:
                        val = 4; break;
                    default:
                        return SR_ERR_ARG;
                }
                sprintf(tmpstr, "t%d%02d\n", val, idx);
                if (send_serial_w_ack_pp(serial, tmpstr) != SR_OK) {
                    sr_err("Trigger cfg to device failed");
                    return SR_ERR;
                }
            }

            sprintf(tmpstr, "p%d\n", devc->pretrig_entries);
            if (send_serial_w_ack_pp(serial, tmpstr) != SR_OK) {
                sr_err("Pretrig to device failed");
                return SR_ERR;
            }

            serial_source_add(sdi->session, serial, G_IO_IN, 200,
                              picopulse_receive, (void *) sdi);

            if (send_serial_str_pp(serial, "C\n") != SR_OK)
                return SR_ERR;
        }
    } else {
        /* 3. Kein Trigger aktiv: Normaler Fixed-Sample-Modus */
        sprintf(tmpstr, "L%" PRIu64 "\n", devc->limit_samples);
        if (send_serial_w_ack_pp(serial, tmpstr) != SR_OK) {
            sr_err("Sample limit to device failed");
            return SR_ERR;
        }

        devc->stl = NULL;
        devc->trigger_fired = TRUE;
        devc->pretrig_entries = 0;
        sr_info("Entering fixed sample mode");
        serial_source_add(sdi->session, serial, G_IO_IN, 200,
                          picopulse_receive, (void *) sdi);

        sprintf(tmpstr, "F\n");
        if (send_serial_str_pp(serial, tmpstr) != SR_OK)
            return SR_ERR;
    }

    std_session_send_df_header(sdi);

    /* NEU: Falls der RP2040 die Rate beim R-Kommando angepasst hat, PulseView informieren */
    sr_session_send_meta(sdi, SR_CONF_SAMPLERATE, g_variant_new_uint64(devc->sample_rate));

    sr_dbg("dsbstartend %d", devc->dig_sample_bytes);

    if (devc->trigger_fired)
        std_session_send_df_trigger(sdi);

    /* Keep this at the end as we don't want to be RX_ACTIVE unless everything
     * is ok */
    devc->rxstate = RX_ACTIVE;

    return SR_OK;
}

/* This function is called either by the protocol code if we reached all of the
 * samples or an error condition, and also by the user clicking stop in
 * pulseview. It must always be called for any acquistion that was started to
 * free memory. */
static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
    struct dev_context *devc;
    struct sr_serial_dev_inst *serial;
    int len;
    devc = sdi->priv;
    serial = sdi->conn;

    sr_dbg("At dev_acquisition_stop");

    std_session_send_df_end(sdi);

    /* If we reached this while still active it is likely because the stop
     * button was pushed in pulseview. That is generally some kind of error
     * condition, so we don't try to check the bytenct */
    if (devc->rxstate == RX_ACTIVE)
        sr_err("Reached dev_acquisition_stop in RX_ACTIVE");

    if (devc->rxstate != RX_IDLE) {
        sr_err("Sending plus to stop device stream");
        send_serial_char_pp(serial, '+');
    }

    /* In case we get calls to receive force it to exit */
    devc->rxstate = RX_IDLE;

    /* Drain data from device so that it doesn't confuse subsequent commands */
    do {
        len = serial_read_blocking(serial, devc->buffer,
                                   devc->serial_buffer_size, 100);
        if (len)
            sr_err("Dropping %d device bytes", len);
    } while (len > 0);

    if (devc->buffer) {
        g_free(devc->buffer);
        devc->buffer = NULL;
    }

    for (int i = 0; i < devc->num_a_channels; i++) {
        if (devc->a_data_bufs[i]) {
            g_free(devc->a_data_bufs[i]);
            devc->a_data_bufs[i] = NULL;
        }
    }
    if (devc->d_data_buf) {
        g_free(devc->d_data_buf);
        devc->d_data_buf = NULL;
    }

    if (devc->stl) {
        soft_trigger_logic_free(devc->stl);
        devc->stl = NULL;
    }

    for (int i = 0; i < devc->num_a_channels; i++) {
        if (devc->a_pretrig_bufs[i])
            g_free(devc->a_pretrig_bufs[i]);
        devc->a_pretrig_bufs[i] = NULL;
    }

    serial = sdi->conn;
    serial_source_remove(sdi->session, serial);

    return SR_OK;
}

static struct sr_dev_driver picopulse_driver_info = {
    .name = "picoPULSE++",
    .longname = "picoPULSE++",
    .api_version = 1,
    .init = std_init,
    .cleanup = std_cleanup,
    .scan = scan,
    .dev_list = std_dev_list,
    .dev_clear = std_dev_clear,
    .config_get = config_get,
    .config_set = config_set,
    .config_list = config_list,
    .dev_open = std_serial_dev_open,
    .dev_close = std_serial_dev_close,
    .dev_acquisition_start = dev_acquisition_start,
    .dev_acquisition_stop = dev_acquisition_stop,
    .context = NULL,
};

SR_REGISTER_DEV_DRIVER(picopulse_driver_info);
