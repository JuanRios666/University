/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

// LoRaWAN region to use, full list of regions can be found at: 
//   http://stackforce.github.io/LoRaMac-doc/LoRaMac-doc-v4.5.1/group___l_o_r_a_m_a_c.html#ga3b9d54f0355b51e85df8b33fd1757eec
#define LORAWAN_REGION                  LORAMAC_REGION_AU915

// LoRaWAN device address (32-bit)
#define LORAWAN_DEV_ADDR                "E1010101"

// LoRaWAN Network Session Key (128-bit)
#define LORAWAN_NETWORK_SESSION_KEY     "CC558C85F6B490AAD9FEA526B79DA864"
  
// LoRaWAN Application Session Key (128-bit)
#define LORAWAN_APP_SESSION_KEY         "F69230260D2FA96BF2E4E114C2C13DB7"

// LoRaWAN Channel Mask, NULL value will use the default channel mask 
// for the region
#define LORAWAN_CHANNEL_MASK            NULL