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
#define _GNU_SOURCE

#include <config.h>
#include <errno.h>
#include <glib.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

SR_PRIV int send_serial_str_pp(struct sr_serial_dev_inst *serial, char *str) {
    int len = strlen(str);
    if ((len > 63) || (len < 1)) {
        sr_err("ERROR: Serial string len %d invalid ", len);
        return SR_ERR;
    }

    /* 100ms timeout. With USB CDC serial we can't define the timeout based
     * on link rate, so just pick something large as we shouldn't normally
     * see them */
    if (serial_write_blocking(serial, str, len, 100) != len) {
        sr_err("ERROR: Serial str write failed");
        return SR_ERR;
    }

    return SR_OK;
}

SR_PRIV int send_serial_char_pp(struct sr_serial_dev_inst *serial, char ch) {
    char buf[1];
    buf[0] = ch;

    if (serial_write_blocking(serial, buf, 1, 100) != 1) {
        /* 100ms */
        sr_err("ERROR: Serial char write failed");
        return SR_ERR;
    }

    return SR_OK;
}

/* Issue a command that expects a string return that is less than 30 characters.
 * Returns the length of string */
int send_serial_w_resp_pp(struct sr_serial_dev_inst *serial, char *str,
                          char *resp, size_t cnt) {
    int num_read, i;
    send_serial_str_pp(serial, str);

    /* Using the serial_read_blocking function when reading a response of
     * unknown length requires a long worst case timeout to always be taken.
     * So, instead loop waiting for a first byte, and then a final small delay
     * for the rest. */
    for (i = 0; i < 1000; i++) {
        /* wait up to 1 second in ms increments */
        num_read = serial_read_blocking(serial, resp, cnt, 1);
        if (num_read > 0)
            break;
    }

    /* Since the serial port is USB CDC we can't calculate timeouts based on
     * baud rate but even if the response is split between two USB transfers,
     * 10ms should be plenty. */
    num_read += serial_read_blocking(serial, &(resp[num_read]), cnt - num_read,
                                     10);
    if ((num_read < 1) || (num_read > 63)) {
        sr_err("ERROR: Serial_w_resp failed (%d).", num_read);
        return -1;
    } else
        return num_read;
}

/* Issue a command that expects a single char ack */
SR_PRIV int send_serial_w_ack_pp(struct sr_serial_dev_inst *serial, char *str) {
    char buf[2];
    int num_read;

    /* In case we have left over transfer from the device, drain them.
     * These should not exist in normal operation */
    while ((num_read = serial_read_blocking(serial, buf, 2, 10)))
        sr_dbg("swack drops 2 bytes %d %d", buf[0], buf[1]);

    send_serial_str_pp(serial, str);

    /* 1000ms timeout */
    num_read = serial_read_blocking(serial, buf, 1, 1000);

    if ((num_read == 1) && (buf[0] == '*')) {
        return SR_OK;
    } else {
        sr_err("ERROR: Serial_w_ack %s failed (%d).", str, num_read);
        if (num_read)
            sr_err("ack resp char %c d %d", buf[0], buf[0]);
        return SR_ERR;
    }
}

/* Universeller RLE-Parser für:
 * - <= 4 Bit  (1 Byte:  1[3b RLE][4b D0..3])
 * - <= 11 Bit (2 Bytes: 1[3b RLE][4b D0..3] 1[7b D4..10])
 * - <= 25 Bit (4 Bytes: 1[3b RLE][4b D0..3] 1[7b D4..10] 1[7b D11..17] 1[7b D18..24])
 * RLE-Erweiterungsbytes haben immer MSB == 0 (ASCII 48..127 -> (byte - 47) * 8). */
void process_rle_pp(struct sr_dev_inst *sdi, struct dev_context *d) {
    uint32_t j, cval;
    uint8_t cbyte;
    uint32_t rlecnt = 0;
    uint32_t sample_bytes;
    uint32_t rle_start_rdptr = d->ser_rdptr;
    int highest_bit = -1;

    /* Höchsten aktivierten Digitalkanal bestimmen (ungenutzte dazwischen sind Dummies) */
    for (int i = 31; i >= 0; i--) {
        if ((d->d_chan_mask >> i) & 1) {
            highest_bit = i;
            break;
        }
    }

    if (highest_bit < 4) {
        sample_bytes = 1; /* 1..4 Bits   -> 1 Byte  */
    } else if (highest_bit < 11) {
        sample_bytes = 2; /* 5..11 Bits  -> 2 Bytes */
    } else if (highest_bit < 25) {
        sample_bytes = 4; /* 12..25 Bits -> 4 Bytes */
    } else {
        sr_err(
            "ERROR: Mehr als 25 Digitalkanaele werden im RLE-Modus nicht unterstuetzt!");
        d->rxstate = RX_ABORT;
        return;
    }

    while (d->ser_rdptr < d->bytes_avail) {
        cbyte = d->buffer[d->ser_rdptr];

        /* 1. Fall: RLE-Zähler-Byte (MSB == 0, Werte 48..127) */
        if ((cbyte >= 48) && (cbyte <= 127)) {
            if (rlecnt == 0)
                rle_start_rdptr = d->ser_rdptr;
            rlecnt += (cbyte - 47) * 8;
            d->byte_cnt++;
            d->ser_rdptr++;
        }
        /* 2. Fall: Neues Sample (MSB == 1) */
        else if (cbyte >= 0x80) {
            /* Prüfen, ob das komplette Sample (1, 2 oder 4 Bytes) im Puffer liegt */
            if ((d->ser_rdptr + sample_bytes) > d->bytes_avail) {
                /* Unvollständiges Sample am Pufferende: ev. RLE-Bytes davor zurückrollen */
                if (rlecnt > 0) {
                    d->byte_cnt -= (d->ser_rdptr - rle_start_rdptr);
                    d->ser_rdptr = rle_start_rdptr;
                    rlecnt = 0;
                }
                break;
            }

            /* Framing-Check: Alle Folgebytes eines Samples müssen MSB == 1 haben */
            gboolean framing_ok = TRUE;
            for (uint32_t b = 1; b < sample_bytes; b++) {
                if (d->buffer[d->ser_rdptr + b] < 0x80) {
                    sr_err("RLE Framing-Error: Folgebyte %u hat MSB=0 (0x%02X)",
                           b, d->buffer[d->ser_rdptr + b]);
                    framing_ok = FALSE;
                    break;
                }
            }
            if (!framing_ok) {
                d->rxstate = RX_ABORT;
                break;
            }

            /* 3-Bit RLE-Zähler aus dem ersten Byte extrahieren */
            rlecnt += (cbyte & 0x70) >> 4;
            if (rlecnt) {
                rle_memset_pp(d, rlecnt);
                rlecnt = 0;
            }

            /* Nutzdaten aus 1, 2 oder 4 Bytes zusammensetzen */
            cval = cbyte & 0x0F;
            if (sample_bytes >= 2) {
                cval |= ((uint32_t) (d->buffer[d->ser_rdptr + 1] & 0x7F)) << 4;
            }
            if (sample_bytes == 4) {
                cval |= ((uint32_t) (d->buffer[d->ser_rdptr + 2] & 0x7F)) << 11;
                cval |= ((uint32_t) (d->buffer[d->ser_rdptr + 3] & 0x7F)) << 18;
            }

            /* Aktuelles Sample als letzten Wert für künftige RLE-Wiederholungen merken */
            d->d_last[0] = cval & 0xFF;
            d->d_last[1] = (cval >> 8) & 0xFF;
            d->d_last[2] = (cval >> 16) & 0xFF;
            d->d_last[3] = (cval >> 24) & 0xFF;

            /* In den sigrok-Ausgabepuffer schreiben */
            uint32_t didx = d->cbuf_wrptr * d->dig_sample_bytes;
            for (j = 0; j < d->dig_sample_bytes; j++) {
                d->d_data_buf[didx + j] = d->d_last[j];
            }

            d->byte_cnt += sample_bytes;
            d->ser_rdptr += sample_bytes;
            d->cbuf_wrptr++;
        }
        /* 3. Fall: Steuerzeichen (< 48, z.B. '$' für reguläres Stream-Ende) */
        else {
            if (rlecnt) {
                rle_memset_pp(d, rlecnt);
                rlecnt = 0;
            }
            if (cbyte == '$') {
                sr_info(
                    "RLE Data stream stops with cbyte %d char %c rdidx %d cnt %lu",
                    cbyte, cbyte, d->ser_rdptr, d->byte_cnt);
                d->rxstate = RX_STOPPED;
            } else {
                sr_err(
                    "RLE Data stream aborts with cbyte %d char %c rdidx %d cnt %lu",
                    cbyte, cbyte, d->ser_rdptr, d->byte_cnt);
                d->rxstate = RX_ABORT;
            }
            break;
        }

        /* Zwischenstand an die sigrok-Session senden, bevor der Puffer voll läuft */
        if ((rlecnt >= 2000) ||
            ((rlecnt + (d->cbuf_wrptr << 2)) > (d->sample_buf_size - 1024))) {
            sr_spew("RLE preoverflow wrptr %d bufsize %d rlecnt %d",
                    d->cbuf_wrptr, d->sample_buf_size, rlecnt);
            rle_memset_pp(d, rlecnt);
            process_group_pp(sdi, d, d->cbuf_wrptr);
            rlecnt = 0;
        }
    }

    /* Restliche RLE-Werte oder Samples am Ende des Empfangsblocks verarbeiten */
    if (rlecnt) {
        sr_spew("Residual RLE slice rlecnt %d", rlecnt);
        rle_memset_pp(d, rlecnt);
    }
    if (d->cbuf_wrptr) {
        sr_spew("Residual RLE data wrptr %d", d->cbuf_wrptr);
        process_group_pp(sdi, d, d->cbuf_wrptr);
    }
}


/* Process incoming data stream and forward to trigger processing with
 * process_group
 * The final value of ser_rdptr indicates how many bytes were processed.
 * This version handles all other enabled channel configurations that
 * Process_D4 doesn't */
void process_slice_pp(struct sr_dev_inst *sdi, struct dev_context *devc) {
    int32_t i;
    uint32_t tmp32, cword;
    uint8_t cbyte;
    uint32_t slice_bytes;

    for (slice_bytes = 1; (slice_bytes < devc->bytes_avail)
                          && (devc->buffer[slice_bytes - 1] >= 0x30);
         slice_bytes++);

    if (slice_bytes != devc->bytes_avail) {
        cbyte = devc->buffer[slice_bytes - 1];
        slice_bytes--;
        if (cbyte == '$') {
            sr_info(
                "Data stream stops with cbyte %d char %c rdidx %d sbytes %d cnt %lu",
                cbyte, cbyte, devc->ser_rdptr, slice_bytes, devc->byte_cnt);
            devc->rxstate = RX_STOPPED;
        } else {
            sr_err(
                "Data stream aborts with cbyte %d char %c rdidx %d sbytes %d cnt %lu",
                cbyte, cbyte, devc->ser_rdptr, slice_bytes, devc->byte_cnt);
            devc->rxstate = RX_ABORT;
        }
    }

    /* If the wrptr is non-zero due to a residual from the previous serial
     * transfer, don't double count it towards byte_cnt*/
    devc->byte_cnt += slice_bytes - (devc->wrptr);

    while (((devc->ser_rdptr + devc->bytes_per_slice) <= slice_bytes)
           || ((devc->ser_rdptr < slice_bytes) &&
               (devc->buffer[devc->ser_rdptr] < 0x80))) {
        if (devc->buffer[devc->ser_rdptr] < 0x80) {
            int16_t rlecnt;
            if (devc->buffer[devc->ser_rdptr] <= 79)
                rlecnt = devc->buffer[devc->ser_rdptr] - 47;
            else
                rlecnt = (devc->buffer[devc->ser_rdptr] - 78) * 32;

            if ((rlecnt >= 1) && (rlecnt <= 1568))
                rle_memset_pp(devc, rlecnt);

            devc->ser_rdptr++;
        } else {
            cword = 0;
            /* Build up a word 7 bits at a time, using only enabled channels */
            for (i = 0; i < devc->num_d_channels; i += 7) {
                if (((devc->d_chan_mask) >> i) & 0x7F) {
                    cword |= ((devc->buffer[devc->ser_rdptr]) & 0x7F) << i;
                    (devc->ser_rdptr)++;
                }
            }

            /* Array-Indizes korrigiert */
            devc->d_last[0] = cword & 0xFF;
            devc->d_last[1] = (cword >> 8) & 0xFF;
            devc->d_last[2] = (cword >> 16) & 0xFF;
            devc->d_last[3] = (cword >> 24) & 0xFF;

            for (i = 0; i < devc->num_d_channels; i += 8) {
                uint32_t idx = ((devc->cbuf_wrptr) * devc->dig_sample_bytes) + (
                                   i >> 3);
                devc->d_data_buf[idx] = cword & 0xFF;
                cword >>= 8;
            }

            /* Each analog value is one or more 7 bit values */
            for (i = 0; i < devc->num_a_channels; i++) {
                if ((devc->a_chan_mask >> i) & 1) {
                    tmp32 = devc->buffer[devc->ser_rdptr] - 0x80;
                    for (int a = 1; a < devc->a_size; a++) {
                        tmp32 += (devc->buffer[(devc->ser_rdptr) + a] - 0x80) <<
                                (7 * a);
                    }
                    devc->a_data_bufs[i][devc->cbuf_wrptr] =
                            ((float) tmp32 * devc->a_scale[i]) + devc->a_offset[
                                i];
                    devc->a_last[i] = devc->a_data_bufs[i][devc->cbuf_wrptr];
                    devc->ser_rdptr += devc->a_size;
                }
            }
            devc->cbuf_wrptr++;
        }

        if ((devc->cbuf_wrptr + 2048) > devc->sample_buf_size) {
            process_group_pp(sdi, devc, devc->cbuf_wrptr);
        }
    }

    if (devc->cbuf_wrptr) {
        process_group_pp(sdi, devc, devc->cbuf_wrptr);
    }
}

/* Send the processed analog values to the session */
int send_analog_pp(struct sr_dev_inst *sdi, struct dev_context *devc,
                   uint32_t num_samples, uint32_t offset) {
    struct sr_datafeed_packet packet;
    struct sr_datafeed_analog analog;
    struct sr_analog_encoding encoding;
    struct sr_analog_meaning meaning;
    struct sr_analog_spec spec;
    struct sr_channel *ch;
    uint32_t i;
    float *fptr;

    sr_analog_init(&analog, &encoding, &meaning, &spec, ANALOG_DIGITS);
    for (i = 0; i < devc->num_a_channels; i++) {
        if ((devc->a_chan_mask >> i) & 1) {
            ch = devc->analog_groups[i]->channels->data;
            analog.meaning->channels =
                    g_slist_append(NULL, ch);
            analog.num_samples = num_samples;
            analog.data = (devc->a_data_bufs[i]) + offset;
            fptr = analog.data;
            sr_spew
            ("send analog num %d offset %d first %f 2 %f",
             num_samples, offset, *(devc->a_data_bufs[i]),
             *fptr);
            analog.meaning->mq = SR_MQ_VOLTAGE;
            analog.meaning->unit = SR_UNIT_VOLT;
            analog.meaning->mqflags = 0;
            packet.type = SR_DF_ANALOG;
            packet.payload = &analog;
            sr_session_send(sdi, &packet);
            g_slist_free(analog.meaning->channels);
        } /* if enabled */
    } /* for channels */
    return 0;
}

/*Send the ring buffer of pre-trigger analog samples.
  The entire buffer is sent (as long as it filled once), but need send two payloads split at the
  the writeptr  */
int send_analog_ring_pp(struct sr_dev_inst *sdi, struct dev_context *devc,
                        uint32_t num_samples) {
    struct sr_datafeed_packet packet;
    struct sr_datafeed_analog analog;
    struct sr_analog_encoding encoding;
    struct sr_analog_meaning meaning;
    struct sr_analog_spec spec;
    struct sr_channel *ch;
    int i;
    uint32_t num_pre, start_pre;
    uint32_t num_post, start_post;
    num_pre =
    (num_samples >=
     devc->pretrig_wr_ptr)
        ? devc->pretrig_wr_ptr
        : num_samples;
    start_pre = devc->pretrig_wr_ptr - num_pre;
    num_post = num_samples - num_pre;
    start_post = devc->pretrig_entries - num_post;
    sr_spew
    ("send_analog ring wrptr %u ns %d npre %u spre %u npost %u spost %u",
     devc->pretrig_wr_ptr, num_samples, num_pre, start_pre,
     num_post, start_post);
    float *fptr;
    sr_analog_init(&analog, &encoding, &meaning, &spec, ANALOG_DIGITS);
    for (i = 0; i < devc->num_a_channels; i++) {
        if ((devc->a_chan_mask >> i) & 1) {
            ch = devc->analog_groups[i]->channels->data;
            analog.meaning->channels =
                    g_slist_append(NULL, ch);
            analog.meaning->mq = SR_MQ_VOLTAGE;
            analog.meaning->unit = SR_UNIT_VOLT;
            analog.meaning->mqflags = 0;
            packet.type = SR_DF_ANALOG;
            packet.payload = &analog;
            /*First send what is after the write pointer because it is oldest */
            if (num_post) {
                analog.num_samples = num_post;
                analog.data =
                        (devc->a_pretrig_bufs[i]) + start_post;
                for (uint32_t j = 0;
                     j < analog.num_samples; j++) {
                    fptr =
                            analog.data +
                            (j * sizeof(float));
                }
                sr_session_send(sdi, &packet);
            }
            if (num_pre) {
                analog.num_samples = num_pre;
                analog.data = (devc->a_pretrig_bufs[i]) + start_pre;
                sr_dbg("Sending A%d ring buffer newest ", i);
                for (uint32_t j = 0; j < analog.num_samples; j++) {
                    fptr = analog.data + (j * sizeof(float));
                    sr_spew("RNGDCW%d j %d %f %p", i, j, *fptr, (void *) fptr);
                }
                sr_session_send(sdi, &packet);
            }
            g_slist_free(analog.meaning->channels);
            sr_dbg("Sending A%d ring buffer done ", i);
        } /*if enabled */
    } /* for channels */
    return 0;
}

/* Given a chunk of slices forward to trigger check or session as appropriate and update state
   these could be real slices or those generated by rles */
int process_group_pp(struct sr_dev_inst *sdi, struct dev_context *devc,
                     uint32_t num_slices) {
    size_t num_samples;
    struct sr_datafeed_logic logic;
    struct sr_datafeed_packet packet;
    int trigger_offset;
    int pre_trigger_samples;

    devc->cbuf_wrptr = 0;

    if (!devc->trigger_fired && num_slices > 0) {
        if (devc->stl) {
            /* Fall 1: Soft-Trigger (<= 15 MHz) auf dem PC auswerten */
            trigger_offset = soft_trigger_logic_check(devc->stl,
                devc->d_data_buf, num_slices * devc->dig_sample_bytes,
                &pre_trigger_samples);

            if (trigger_offset > -1) {
                devc->trigger_fired = TRUE;

                /* Analoge Samples vor trigger_offset noch in den Ringpuffer übernehmen */
                if (devc->pretrig_entries > 0) {
                    for (int s = 0; s < trigger_offset; s++) {
                        for (uint32_t ch = 0; ch < devc->num_a_channels; ch++) {
                            if ((devc->a_chan_mask >> ch) & 1) {
                                devc->a_pretrig_bufs[ch][devc->pretrig_wr_ptr] =
                                        devc->a_data_bufs[ch][s];
                            }
                        }
                        devc->pretrig_wr_ptr =
                                (devc->pretrig_wr_ptr + 1) % devc->
                                pretrig_entries;
                    }
                }

                /* soft_trigger_logic_check sendet pre_trigger_samples + trigger_offset Logik-Samples */
                uint32_t total_pretrig = pre_trigger_samples + trigger_offset;
                if (total_pretrig > devc->pretrig_entries) {
                    total_pretrig = devc->pretrig_entries;
                }
                devc->sent_samples += total_pretrig;

                /* Gespeicherte analoge Pre-Trigger-Samples synchron senden */
                if (devc->pretrig_entries > 0 && total_pretrig > 0) {
                    send_analog_ring_pp(sdi, devc, total_pretrig);
                }

                /* Restliche Samples ab dem Trigger-Punkt im aktuellen Block senden */
                num_samples = num_slices - trigger_offset;
                if (devc->limit_samples &&
                    num_samples > devc->limit_samples - devc->sent_samples) {
                    num_samples = devc->limit_samples - devc->sent_samples;
                }

                if (num_samples > 0) {
                    if (devc->num_d_channels) {
                        packet.type = SR_DF_LOGIC;
                        packet.payload = &logic;
                        logic.unitsize = devc->dig_sample_bytes;
                        logic.length = num_samples * logic.unitsize;
                        logic.data = devc->d_data_buf +
                                     (trigger_offset * devc->dig_sample_bytes);
                        sr_session_send(sdi, &packet);
                    }
                    send_analog_pp(sdi, devc, num_samples, trigger_offset);
                    devc->sent_samples += num_samples;
                }
            } else if (devc->pretrig_entries > 0) {
                /* Trigger noch nicht ausgelöst: analoge Samples im Ringpuffer merken */
                for (uint32_t s = 0; s < num_slices; s++) {
                    for (uint32_t ch = 0; ch < devc->num_a_channels; ch++) {
                        if ((devc->a_chan_mask >> ch) & 1) {
                            devc->a_pretrig_bufs[ch][devc->pretrig_wr_ptr] =
                                    devc->a_data_bufs[ch][s];
                        }
                    }
                    devc->pretrig_wr_ptr =
                            (devc->pretrig_wr_ptr + 1) % devc->pretrig_entries;
                }
            }
            return 0;
        } else {
            /* Fall 2: Hardware-Trigger (> 15 MHz) vom RP2040 */
            devc->sent_samples = 0;
            std_session_send_df_trigger(sdi);
            devc->trigger_fired = TRUE;
            sr_info("HW Trigger on Target detected! Processing linear stream.");
        }
    }

    if (devc->limit_samples &&
        num_slices > devc->limit_samples - devc->sent_samples) {
        num_samples = devc->limit_samples - devc->sent_samples;
    } else {
        num_samples = num_slices;
    }

    if (num_samples > 0) {
        sr_spew("Process_group sending %lu samples directly to session",
                num_samples);
        if (devc->num_d_channels) {
            packet.type = SR_DF_LOGIC;
            packet.payload = &logic;
            logic.unitsize = devc->dig_sample_bytes;
            logic.length = num_samples * logic.unitsize;
            logic.data = devc->d_data_buf;
            sr_session_send(sdi, &packet);
        }
        send_analog_pp(sdi, devc, num_samples, 0);
    }

    devc->sent_samples += num_samples;
    return 0;
}

/* Duplicate previous sample values
 * This function relies on the caller to ensure d_data_buf has samples to handle
 * the full value of the rle */
void rle_memset_pp(struct dev_context *devc, uint32_t num_slices) {
    uint32_t j, k, didx;
    sr_spew("rle_memset_pp vals 0x%X, 0x%X, 0x%X slices %d dsb %d",
            devc->d_last[0], devc->d_last[1], devc->d_last[2],
            num_slices, devc->dig_sample_bytes);

    /* Even if a channel is disabled, PV expects the same location and size for
     * the enabled channels as if the channel were enabled. */
    for (j = 0; j < num_slices; j++) {
        didx = devc->cbuf_wrptr * devc->dig_sample_bytes;
        for (k = 0; k < devc->dig_sample_bytes; k++)
            devc->d_data_buf[didx + k] = devc->d_last[k];
        /* cbuf_wrptr always counts slices/samples (and not the bytes in the
         * buffer) regardless of mode */
        devc->cbuf_wrptr++;
    }
}

/* This callback function is mapped from api.c with serial_source_add and is
 * created after a capture has been setup and is responsible for querying the
 * device trigger status, downloading data and forwarding packets */
SR_PRIV int picopulse_receive(int fd, int revents, void *cb_data) {
    struct sr_dev_inst *sdi;
    struct dev_context *devc;
    struct sr_serial_dev_inst *serial;
    int len;
    uint32_t i, bytes_rem, residual_bytes;
    (void) fd;

    if (!(sdi = cb_data))
        return TRUE;

    if (!(devc = sdi->priv))
        return TRUE;

    if (devc->rxstate != RX_ACTIVE) {
        /* This condition is normal operation and expected to happen
         * but printed as information */
        sr_dbg("Reached non active state in receive %d", devc->rxstate);
        /* Don't return - we may be waiting for a final bytecnt */
    }

    if (devc->rxstate == RX_IDLE) {
        /* This is the normal end condition where we do one more receive
         * to make sure we get the full byte_cnt */
        sr_dbg("Reached idle state in receive %d", devc->rxstate);
        return FALSE;
    }

    serial = sdi->conn;

    /* Return true if it is some kind of event we don't handle */
    if (!(revents == G_IO_IN || revents == 0))
        return TRUE;

    /* Fill the buffer, note the end may have partial slices */
    bytes_rem = devc->serial_buffer_size - devc->wrptr;

    /* Read one byte less so that we can null it and print as a string. Do a
     * small 10ms timeout, if we get nothing, we'll always come back again */
    len = serial_read_blocking(serial, &(devc->buffer[devc->wrptr]),
                               bytes_rem - 1, 10);
    sr_spew("Entry wrptr %u bytes_rem %u len %d", devc->wrptr, bytes_rem, len);

    if (len > 0) {
        devc->buffer[devc->wrptr + len] = 0;
        /* Add the "#" so that spaces in the string are clearly seen */
        sr_dbg("rx string %s#", devc->buffer);
        devc->bytes_avail = (devc->wrptr + len);
        sr_spew("rx len %d bytes_avail %ul sent_samples %ul wrptr %u",
                len, devc->bytes_avail, devc->sent_samples, devc->wrptr);
    } else {
        if (len == 0) {
            return TRUE;
        } else {
            sr_err("ERROR: Negative serial read code %d", len);
            sdi->driver->dev_acquisition_stop(sdi);
            return FALSE;
        }
    }

    /* Process the serial read data */
    devc->ser_rdptr = 0;
    if (devc->rxstate == RX_ACTIVE) {
        if ((devc->a_chan_mask == 0) && ((devc->d_chan_mask & 0xFE000000) == 0))
            process_rle_pp(sdi, devc);
        else
            process_slice_pp(sdi, devc);
    }


    /* process_slice/process_D4 increment ser_rdptr as bytes of the serial
     * buffer are used. But they may not use all of it, and thus the residual
     * unused bytes are shifted to the start of the buffer for the next call. */
    residual_bytes = devc->bytes_avail - devc->ser_rdptr;
    if (residual_bytes) {
        for (i = 0; i < residual_bytes; i++)
            devc->buffer[i] = devc->buffer[i + devc->ser_rdptr];

        devc->ser_rdptr = 0;
        devc->wrptr = residual_bytes;
        sr_spew("Residual shift rdptr %u wrptr %u", devc->ser_rdptr,
                devc->wrptr);
    } else {
        /* If there are no residuals shifted then zero the wrptr since all data
         * is used */
        devc->wrptr = 0;
    }

    /* ABORT ends immediately */
    if (devc->rxstate == RX_ABORT) {
        sr_err("Ending receive on abort");
        sdi->driver->dev_acquisition_stop(sdi);
        return FALSE;
    }

    /* If stopped, look for '$' and '+' indicating the full byte_cnt is received */
    if (devc->rxstate == RX_STOPPED) {
        sr_dbg("Stopped, checking byte_cnt");
        char *dollar = memchr(devc->buffer, '$', devc->wrptr);
        if (dollar) {
            size_t offset = dollar - (char *)devc->buffer;
            char *plus = memchr(dollar + 1, '+', devc->wrptr - offset - 1);
            if (plus) {
                *plus = 0;
                uint64_t rxbytecnt = strtoull(dollar + 1, NULL, 10);
                sr_dbg("Byte_cnt check device cnt %" PRIu64 " host cnt %" PRIu64,
                       rxbytecnt, devc->byte_cnt);
                if (rxbytecnt != devc->byte_cnt)
                    sr_err("ERROR: received %" PRIu64 " and counted %" PRIu64 " bytecnts "
                           "don't match, data may be lost",
                           rxbytecnt, devc->byte_cnt);

                /* Since we got the bytecnt we know the device is done sending data */
                devc->rxstate = RX_IDLE;

                /* We must always call acquisition_stop on all completed runs */
                sdi->driver->dev_acquisition_stop(sdi);
                return FALSE;
            }
        }
        sr_dbg("Haven't seen complete $byte_cnt+ yet");
    }

    /* If at the sample limit, send a "+" in case we are in continuous mode and
     * need to stop the device. */
    if ((devc->sent_samples >= devc->limit_samples)
        && (devc->rxstate == RX_ACTIVE)) {
        sr_dbg("Ending: sent %u of limit %" PRIu64 " samples byte_cnt %" PRIu64,
               devc->sent_samples, devc->limit_samples, devc->byte_cnt);
        send_serial_char_pp(serial, '+');
        devc->rxstate = RX_IDLE;
        sdi->driver->dev_acquisition_stop(sdi);
        return FALSE;
    }

    sr_spew("Receive function done: sent %u limit %lu wrptr %u len %d",
            devc->sent_samples, devc->limit_samples, devc->wrptr, len);

    return TRUE;
}

/* Read device specific information from the device */
SR_PRIV int picopulse_get_dev_cfg(const struct sr_dev_inst *sdi) {
    struct dev_context *devc;
    struct sr_serial_dev_inst *serial;
    char *cmd, response[20];
    gchar **tokens;
    unsigned int i;
    int ret, num_tokens;

    devc = sdi->priv;
    sr_dbg("At get_dev_cfg");
    serial = sdi->conn;
    for (i = 0; i < devc->num_a_channels; i++) {
        cmd = g_strdup_printf("a%d\n", i);
        ret = send_serial_w_resp_pp(serial, cmd, response, 20);
        if (ret <= 0) {
            sr_err("ERROR: No response from device for analog channel query");
            return SR_ERR;
        }
        response[ret] = 0;
        tokens = NULL;
        tokens = g_strsplit(response, "x", 0);
        num_tokens = g_strv_length(tokens);

        if (num_tokens == 2) {
            devc->a_scale[i] = ((float) atoi(tokens[0])) / 1000000.0;
            devc->a_offset[i] = ((float) atoi(tokens[1])) / 1000000.0;
            sr_dbg("A%d scale %f offset %f response #%s# tokens #%s# #%s#",
                   i, devc->a_scale[i], devc->a_offset[i],
                   response, tokens[0], tokens[1]);
        } else {
            sr_err(
                "ERROR: Ascale read c%d got unparseable response %s tokens %d",
                i, response, num_tokens);
            /* Force a legal fixed value assuming a 3.3V scale */
            devc->a_scale[i] = 0.0257;
            devc->a_offset[i] = 0.0;
        }

        g_strfreev(tokens);
        g_free(cmd);
    }

    return SR_OK;
}
