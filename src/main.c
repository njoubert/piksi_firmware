/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "init.h"
#include "main.h"
#include "cw.h"
#include "sbp.h"
#include "board/nap/nap_common.h"
#include "board/nap/nap_conf.h"
#include "board/nap/track_channel.h"
#include "track.h"
#include "acq.h"
#include "nmea.h"
#include "manage.h"
#include "timing.h"
#include "position.h"
#include "peripherals/spi.h"
#include "board/leds.h"
#include "board/m25_flash.h"

#include <libswiftnav/pvt.h>
#include <libswiftnav/sbp.h>
#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/rtcm3.h>
#include <libswiftnav/single_diff.h>
#include <libswiftnav/dgnss_management.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>

#include "settings.h"

channel_measurement_t meas[MAX_CHANNELS];
navigation_measurement_t nav_meas[MAX_CHANNELS];
navigation_measurement_t nav_meas_old[MAX_CHANNELS];

navigation_measurement_t nav_meas_base[MAX_CHANNELS];
u8 n_base;
double tow_base = -1;
navigation_measurement_t nav_meas_rover[MAX_CHANNELS];
u8 n_rover;
gps_time_t t_rover;
u8 SOLN_FREQ = 3;
u8 filter_choice = 1;
double b_init[3] = {1.02571973, -0.15447333, 0.81029273}; // The antenna tree

int _getpid()
{
  return 1;
}
void _exit(int rc)
{
  (void)rc;
  while(1);
}
int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  return -1; /* Always fails */
}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  /*u16 id;*/
  /*u8 sync;*/
  /*rtcm3_decode_1002(msg, &id, &tow_base, &n_base, nav_meas_base, &sync);*/

  tow_base = ((gps_time_t *)msg)->tow;
  n_base = (len - sizeof(gps_time_t)) / sizeof(msg_obs_t);
  msg_obs_t *obs = (msg_obs_t *)(msg + sizeof(gps_time_t));
  for (u8 i=0; i<n_base; i++) {
    nav_meas_base[i].prn = obs[i].prn;
    nav_meas_base[i].raw_pseudorange = obs[i].P;
    nav_meas_base[i].carrier_phase = obs[i].L;
    nav_meas_base[i].snr = obs[i].snr;
  }

  /* Ensure observations sorted by PRN. */
  qsort(nav_meas_base, n_base, sizeof(navigation_measurement_t), nav_meas_cmp);

  static u32 obs_count = 0;
  obs_count++;
  if (obs_count % 20 == 0) {
    printf("Obs count: %u\n", (unsigned int)obs_count);
  }
}

void send_observations(gps_time_t t, u8 n, navigation_measurement_t *m)
{
  u8 buff[256];
  /*u16 len = rtcm3_encode_1002(buff, 0, t, n, m, 0);*/
  /*sbp_send_msg(MSG_NEW_OBS, len, buff);*/

  memcpy(buff, &t, sizeof(gps_time_t));
  msg_obs_t *obs = (msg_obs_t *)&buff[sizeof(gps_time_t)];
  if (n * sizeof(msg_obs_t) > 255 - sizeof(gps_time_t))
    n = 255 / sizeof(msg_obs_t);
  for (u8 i=0; i<n; i++) {
    obs[i].prn = m[i].prn;
    obs[i].P = m[i].raw_pseudorange;
    obs[i].L = m[i].carrier_phase;
    obs[i].snr = m[i].snr;
  }
  sbp_send_msg(MSG_NEW_OBS, sizeof(gps_time_t) + n*sizeof(msg_obs_t), buff);
}

/* TODO: Think about thread safety when updating ephemerides. */
ephemeris_t es[MAX_SATS];
ephemeris_t es_old[MAX_SATS];

void tim5_isr()
{
  led_toggle(LED_RED);

  u8 n_ready = 0;
  for (u8 i=0; i<nap_track_n_channels; i++) {
    if (tracking_channel[i].state == TRACKING_RUNNING && \
        es[tracking_channel[i].prn].valid == 1 && \
        es[tracking_channel[i].prn].healthy == 1 && \
        tracking_channel[i].update_count \
          - tracking_channel[i].snr_below_threshold_count > TRACK_SNR_THRES_COUNT && \
        tracking_channel[i].TOW_ms > 0) {
      __asm__("CPSID i;");
      tracking_update_measurement(i, &meas[n_ready]);
      __asm__("CPSIE i;");

      if (meas[n_ready].snr > 2)
        n_ready++;
    }
  }

  if (n_ready >= 4) {
    /* Got enough sats/ephemerides, do a solution. */
    /* TODO: Instead of passing 32 LSBs of nap_timing_count do something
     * more intelligent with the solution time.
     */
    static u8 n_ready_old = 0;
    u64 nav_tc = nap_timing_count();

    calc_navigation_measurement(n_ready, meas, nav_meas, (double)((u32)nav_tc)/SAMPLE_FREQ, es);

    n_rover = tdcp_doppler(n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_rover);

    dops_t dops;
    if (calc_PVT(n_rover, nav_meas_rover, &position_solution, &dops) == 0) {
      position_updated();


      double expected_tow = round(position_solution.time.tow*(double)SOLN_FREQ) / (double)SOLN_FREQ;
      double t_err = expected_tow - position_solution.time.tow;

      for (u8 i=0; i<n_rover; i++) {
        //nav_meas_rover[i].pseudorange = gpsdifftime(nav_meas_rover[i].tot, position_solution.time) * GPS_C;
        nav_meas_rover[i].pseudorange -= t_err * nav_meas_rover[i].doppler * (GPS_C / GPS_L1_HZ);
        nav_meas_rover[i].carrier_phase += t_err * nav_meas_rover[i].doppler;
        if (fabs(t_err) > 0.01)
          printf("dphase[%d] = %f * %f = %f\n", i, t_err, nav_meas_rover[i].doppler, t_err * nav_meas_rover[i].doppler);
      }
      t_rover.wn = position_solution.time.wn;
      t_rover.tow = expected_tow;

      send_observations(t_rover, n_rover, nav_meas_rover);

      double dt = expected_tow + (1.0/SOLN_FREQ) - position_solution.time.tow;

      /* Limit dt to 2 seconds maximum to prevent hang if dt calculated incorrectly. */
      if (dt > 2)
        dt = 2;

      timer_set_period(TIM5, round(65472000 * dt));

      if (!settings.simulation_mode) {
      sbp_gps_time_t gps_time;
      gps_time.wn = position_solution.time.wn;
      gps_time.tow = round(position_solution.time.tow * 1e3);
      gps_time.ns = round((position_solution.time.tow - gps_time.tow*1e-3) * 1e9);
      gps_time.flags = 0;
      sbp_send_msg(SBP_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);

      sbp_pos_llh_t pos_llh;
      pos_llh.tow = round(position_solution.time.tow * 1e3);
      pos_llh.lat = position_solution.pos_llh[0] * R2D;
      pos_llh.lon = position_solution.pos_llh[1] * R2D;
      pos_llh.height = position_solution.pos_llh[2];
      pos_llh.h_accuracy = 0;
      pos_llh.v_accuracy = 0;
      pos_llh.n_sats = n_rover;
      pos_llh.flags = 0;
      sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

      sbp_vel_ned_t vel_ned;
      vel_ned.tow = round(position_solution.time.tow * 1e3);
      vel_ned.n = round(position_solution.vel_ned[0] * 1e3);
      vel_ned.e = round(position_solution.vel_ned[1] * 1e3);
      vel_ned.d = round(position_solution.vel_ned[2] * 1e3);
      vel_ned.h_accuracy = 0;
      vel_ned.v_accuracy = 0;
      vel_ned.n_sats = n_rover;
      vel_ned.flags = 0;
      sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

      nmea_gpgga(&position_solution, &dops);

      DO_EVERY(10,
        sbp_dops_t sbp_dops;
        sbp_dops.pdop = round(dops.pdop * 100);
        sbp_dops.gdop = round(dops.gdop * 100);
        sbp_dops.tdop = round(dops.tdop * 100);
        sbp_dops.hdop = round(dops.hdop * 100);
        sbp_dops.vdop = round(dops.vdop * 100);
        sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
        nmea_gpgsv(n_rover, nav_meas_rover, &position_solution);
      );
      }
    }
    memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
    n_ready_old = n_ready;
  }

  //Here we do all the nice simulation-related stuff.
  if (settings.simulation_mode > 0) {

    //Set the timer period appropriately
    timer_set_period(TIM5, round(65472000 * (1.0/SOLN_FREQ)));

    //First we propagate the current fake PVT solution
    u32 now_ticks = time_ticks();
    if (settings.simulation_last_update_ticks == 0) {

      //If this is the first time we see this, just set our tick count.
      //This prevents a big jump at the first message.
      settings.simulation_last_update_ticks = now_ticks;

    } else {
      
      double elapsed_seconds = (now_ticks - settings.simulation_last_update_ticks)/(double)TICK_FREQ;
      settings.simulation_last_update_ticks = now_ticks;

      settings.simulation_tow += 1000.0*elapsed_seconds;

      double vel_ned_given[3] = {1.0,0.0,0.0};
      double vel_ecef[3];
      wgsned2ecef(vel_ned_given, settings.simulation_current_ecef, vel_ecef);

      settings.simulation_current_ecef[0] += vel_ecef[0]*elapsed_seconds;
      settings.simulation_current_ecef[1] += vel_ecef[1]*elapsed_seconds;
      settings.simulation_current_ecef[2] += vel_ecef[2]*elapsed_seconds;

      double current_pos_llh[3];
      wgsecef2llh(settings.simulation_current_ecef, current_pos_llh);
      current_pos_llh[0] = current_pos_llh[0]*R2D;
      current_pos_llh[1] = current_pos_llh[1]*R2D;

      //Then we send fake messages

      sbp_gps_time_t gps_time;
      gps_time.wn = settings.simulation_wn;
      gps_time.tow = settings.simulation_tow;
      gps_time.ns = 0;
      gps_time.flags = 0;
      sbp_send_msg(SBP_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);

      sbp_pos_llh_t pos_llh;
      pos_llh.tow = settings.simulation_tow;
      pos_llh.lat = current_pos_llh[0];
      pos_llh.lon = current_pos_llh[1];
      pos_llh.height = current_pos_llh[2];
      pos_llh.h_accuracy = 0;
      pos_llh.v_accuracy = 0;
      pos_llh.n_sats = 9;
      pos_llh.flags = 0;
      sbp_send_msg(SBP_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);

      sbp_vel_ned_t vel_ned;
      vel_ned.tow = settings.simulation_tow;
      vel_ned.n = (s32) vel_ned_given[0]*1000;
      vel_ned.e = (s32) vel_ned_given[1]*1000;
      vel_ned.d = (s32) vel_ned_given[2]*1000;
      vel_ned.h_accuracy = 0;
      vel_ned.v_accuracy = 0;
      vel_ned.n_sats = 9;
      vel_ned.flags = 0;
      sbp_send_msg(SBP_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);

      DO_EVERY(10,
        sbp_dops_t sbp_dops;
        sbp_dops.pdop = round(1.9 * 100);
        sbp_dops.gdop = round(1.8 * 100);
        sbp_dops.tdop = round(1.7 * 100);
        sbp_dops.hdop = round(1.6 * 100);
        sbp_dops.vdop = round(1.5 * 100);
        sbp_send_msg(SBP_DOPS, sizeof(sbp_dops_t), (u8 *) &sbp_dops);
      );


    }


  }

  timer_clear_flag(TIM5, TIM_SR_UIF);
}

void timer_setup()
{
  /* Enable TIM5 clock. */
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM5EN);
  nvic_enable_irq(NVIC_TIM5_IRQ);
  nvic_set_priority(NVIC_TIM5_IRQ, 22);
  nvic_set_priority(NVIC_EXTI1_IRQ, 1);
  timer_reset(TIM5);
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM5, 0);
  timer_disable_preload(TIM5);
  timer_set_period(TIM5, 65472000); /* 1 second. */
  timer_enable_counter(TIM5);
  timer_enable_irq(TIM5, TIM_DIER_UIE);
}

void send_baseline(gps_time_t t, u8 n_sats, double b_ecef[3], double ref_ecef[3])
{
  if (1) {
    sbp_baseline_ecef_t sbp_ecef = {
      .tow = t.tow,
      .x = (s32)round(1e3 * b_ecef[0]),
      .y = (s32)round(1e3 * b_ecef[1]),
      .z = (s32)round(1e3 * b_ecef[2]),
      .n_sats = n_sats,
    };
    sbp_send_msg(SBP_BASELINE_ECEF, sizeof(sbp_ecef), (u8 *)&sbp_ecef);
  }

  if (1) {
    double b_ned[3];
    wgsecef2ned(b_ecef, ref_ecef, b_ned);

    sbp_baseline_ned_t sbp_ned = {
      .tow = t.tow,
      .n = (s32)round(1e3 * b_ned[0]),
      .e = (s32)round(1e3 * b_ned[1]),
      .d = (s32)round(1e3 * b_ned[2]),
      .n_sats = n_sats,
    };
    sbp_send_msg(SBP_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);
  }
}


//********************************************************************
// Hacky settings - should be replaced by proper settings subsystem
//********************************************************************
u8 init_done = 0;

void rejig_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  //RESETS BOTH FILTERs, connected to init ambs button in console
  //implemented in the gui code in baseline_view.py
  (void)sender_id; (void)len; (void) context;
  init_done = msg[0];
}
void filter_select_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  //Send it either a 0 or 1 to choose fakeRTK or float
  // 0 == stupid filter
  // 1 == kalman float filter
  (void)sender_id; (void)len; (void) context;
  filter_choice = msg[0];
  printf("FILTER CHANGED: %d\n", filter_choice);
}
void freq_set_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  //Changes the solution frequency
  (void)sender_id; (void)len; (void) context;
  SOLN_FREQ = msg[0];
  printf("FREQ CHANGED: %d\n", SOLN_FREQ);
}
void set_init_b_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  //Initializes a baseline for the fakeRTK

  (void)sender_id; (void)len; (void) context;
  memcpy(b_init, msg, 3*sizeof(double));
  printf("B INIT CHANGED: %f, %f, %f\n", b_init[0], b_init[1], b_init[2]);
}

void set_obs_on_off_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  //Turns on/off whether this piksi transmits anything on UARTA
  (void)sender_id; (void)len; (void) context; (void) msg;
  if (settings.uarta_usart.message_mask) {
    settings.uarta_usart.message_mask = 0x00;
    printf("uarta_usart.message_mask = 0x00\n");

  } else {
    settings.uarta_usart.message_mask = 0x40;
    printf("uarta_usart.message_mask = 0x40\n");

  }
}

void set_simulation_mode_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  //Turns simulation mode on/off
  (void)sender_id; (void)len; (void) context;
  settings.simulation_mode = msg[0];
  if (settings.simulation_mode > 0) {
    led_on(LED_GREEN);
  } else {
    led_off(LED_GREEN);
  }
  printf("Simulation Mode: %d\n", settings.simulation_mode);

}
//********************************************************************
// END Hacky settings
//********************************************************************

int main(void)
{
  init(1);

  printf("\n\nFirmware info - git: " GIT_VERSION ", built: " __DATE__ " " __TIME__ "\n");
  u8 nap_git_hash[20];
  nap_conf_rd_git_hash(nap_git_hash);
  printf("SwiftNAP git: ");
  for (u8 i=0; i<20; i++)
    printf("%02x", nap_git_hash[i]);
  if (nap_conf_rd_git_unclean())
    printf(" (unclean)");
  printf("\n");
  printf("SwiftNAP configured with %d tracking channels\n\n", nap_track_n_channels);

  cw_setup();
  manage_acq_setup();
  tick_timer_setup();
  timing_setup();
  position_setup();
  timer_setup();

  static sbp_msg_callbacks_node_t rejig_node;
  sbp_register_cbk(
    0x99,
    &rejig_callback,
    &rejig_node
  );
  static sbp_msg_callbacks_node_t filter_select_node;
  sbp_register_cbk(
    0x98,
    &filter_select_callback,
    &filter_select_node
  );
  static sbp_msg_callbacks_node_t freq_set_node;
  sbp_register_cbk(
    0x97,
    &freq_set_callback,
    &freq_set_node
  );
  static sbp_msg_callbacks_node_t set_init_b_node;
  sbp_register_cbk(
    0x96,
    &set_init_b_callback,
    &set_init_b_node
  );
  static sbp_msg_callbacks_node_t set_obs_on_off_node;
  sbp_register_cbk(
    0x95,
    &set_obs_on_off_callback,
    &set_obs_on_off_node
  );

  static sbp_msg_callbacks_node_t obs_node;
  sbp_register_cbk(
    MSG_NEW_OBS,
    &obs_callback,
    &obs_node
  );

  static sbp_msg_callbacks_node_t set_simulation_mode_node;
  sbp_register_cbk(
    0x94,
    &set_simulation_mode_callback,
    &set_simulation_mode_node
  );

  while(1)
  {
    sbp_process_messages();
    manage_track();
    manage_acq();

    /* Check if there is a new nav msg subframe to process.
     * TODO: move this into a function */

    memcpy(es_old, es, sizeof(es));
    for (u8 i=0; i<nap_track_n_channels; i++) {
      if (tracking_channel[i].state == TRACKING_RUNNING && tracking_channel[i].nav_msg.subframe_start_index) {
        s8 ret = process_subframe(&tracking_channel[i].nav_msg, &es[tracking_channel[i].prn]);
        if (ret < 0)
          printf("PRN %02d ret %d\n", tracking_channel[i].prn+1, ret);

        if (ret == 1 && !es[tracking_channel[i].prn].healthy)
          printf("PRN %02d unhealthy\n", tracking_channel[i].prn+1);
        if (memcmp(&es[tracking_channel[i].prn], &es_old[tracking_channel[i].prn], sizeof(ephemeris_t))) {
          printf("New ephemeris for PRN %02d\n", tracking_channel[i].prn+1);
          /* TODO: This is a janky way to set the time... */
          gps_time_t t;
          t.wn = es[tracking_channel[i].prn].toe.wn;
          t.tow = tracking_channel[i].TOW_ms / 1000.0;
          if (gpsdifftime(t, es[tracking_channel[i].prn].toe) > 2*24*3600)
            t.wn--;
          else if (gpsdifftime(t, es[tracking_channel[i].prn].toe) < 2*24*3600)
            t.wn++;
          /*set_time(TIME_COARSE, t);*/
        }
      }
    }

    static double ref_ecef[3] = {-2703997.584, -4262084.36, 3886179.86};
    /*static double b_init[3] = {0, 0, 0};*/

    static double last_tow = -1;
    if (fabs(t_rover.tow - tow_base) < 10e-3) {
      if (tow_base != last_tow) {
        static sdiff_t sds[MAX_CHANNELS];
        u8 n_sds = single_diff(
            n_rover, nav_meas_rover,
            n_base, nav_meas_base,
            sds
        );
        if (n_sds > 4) {
          if (init_done == 0) {
            printf("====== INIT =======\n");
            dgnss_init(n_sds, sds, ref_ecef, b_init, 1.0 / SOLN_FREQ);
            init_done = 1;
          }

          double b[3];
          /*u32 t = timer_get_counter(TIM5);*/
          dgnss_update(n_sds, sds, ref_ecef, 1.0 / SOLN_FREQ, filter_choice, b);
          /*u32 dt = timer_get_counter(TIM5) - t;*/
          send_baseline(t_rover, n_sds, b, ref_ecef);

          /*printf("b_ecef: %.3f %.3f %.3f\nDT: %lu\n", b[0], b[1], b[2], dt);*/
          /*double db[3];*/
          /*vector_subtract(3, b, b_init, db);*/
          /*printf("db (cm): %.3f %.3f %.3f (%.3f)\n",*/
              /*100*db[0], 100*db[1], 100*db[2], vector_norm(3, db)*100);*/
        }

        last_tow = tow_base;
      }
    }

    DO_EVERY_TICKS(TICK_FREQ,
      nmea_gpgsa(tracking_channel, 0);
    );
    DO_EVERY_TICKS(TICK_FREQ/5, // 5 Hz update
      tracking_send_state();
    );
    DO_EVERY_TICKS(TICK_FREQ*3,
      if (crc_errors > 0) {
        printf("CRC error count: %u\n", (unsigned int)crc_errors);        
      }
      crc_errors = 0;
    );

    u32 err = nap_error_rd_blocking();
    if (err)
      printf("Error: 0x%08X\n", (unsigned int)err);
  }

  while (1);

	return 0;
}





