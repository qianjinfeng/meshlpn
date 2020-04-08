/* Copyright (c) 2010 - 2019, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ALI_BUT_COMMON_H__
#define ALI_BUT_COMMON_H__

#include <stdint.h>
#include "access.h"

/**
 * @defgroup ALI_BUT_MODEL Ali Wireless Button model
 * This model implements the message based interface required to
 * set the 1 bit value on the server.
 *
 * This implementation of a simple OnOff model can be used to switch things
 * on or off by manipulating a single on/off state. The intention of this model
 * is to have a simple example model that can be used as a baseline for constructing
 * your own model.
 *
 * Do not confuse the simple OnOff model with the Generic OnOff Model specified
 * in the Mesh Model Specification v1.0. The Generic OnOff Model provides additional
 * features such as control over when and for how long the transition between
 * the on/off state should be performed.
 *
 * @note When the server has a publish address set (as in the light switch example),
 * the server will publish its state to its publish address every time its state changes.
 *
 * For more information about creating models, see
 * @ref md_doc_libraries_how_to_models.
 *
 * Model Identification
 * @par
 * Company ID: @ref ALI_BUT_COMPANY_ID
 * @par
 * Ali Wireless Button Client Model ID: @ref ALI_BUT_CLIENT_MODEL_ID
 * @par
 * Ali Wireless Button Server Model ID: @ref ALI_BUT_SERVER_MODEL_ID
 *
 * List of supported messages:
 * @par
 * @copydoc ALI_BUT_OPCODE_SET
 * @par
 * @copydoc ALI_BUT_OPCODE_GET
 * @par
 * @copydoc ALI_BUT_OPCODE_SET_UNRELIABLE
 * @par
 * @copydoc ALI_BUT_OPCODE_STATUS
 *
 * @ingroup MESH_API_GROUP_VENDOR_MODELS
 * @{
 * @defgroup ALI_BUT_COMMON Common Ali Wireless Button definitions
 * Types and definitions shared between the two Ali Wireless Button models.
 * @{
 */

/*lint -align_max(push) -align_max(1) */

/** Vendor specific company ID for Ali Wireless Button model */
#define ALI_BUT_COMPANY_ID    0x01A8

/** Ali Wireless Button opcodes. */
typedef enum
{
    ALI_BUT_OPCODE_GET = 0xD0,            /**< Vendor Message Attr Get. */
    ALI_BUT_OPCODE_STATUS = 0xD3          /**< Vendor Message Attr Status. */
} ali_but_opcode_t;


/** Message format for the Ali Wireless Button Status message. */
typedef struct __attribute((packed))
{
    uint8_t tid;
    uint16_t attr;       /**< 0xF009 keyClickEvent . */
    uint16_t value;
} ali_but_msg_status_t;

/*lint -align_max(pop) */

/** @} end of ALI_BUT_COMMON */
/** @} end of ALI_BUT_MODEL */
#endif /* ALI_BUT_COMMON_H__ */
