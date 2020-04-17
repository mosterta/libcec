#pragma once
/*
 * This file is part of the libCEC(R) library.
 *
 * libCEC(R) is Copyright (C) 2011-2013 Pulse-Eight Limited.  All rights reserved.
 * libCEC(R) is an original work, containing original code.
 *
 * libCEC(R) is a trademark of Pulse-Eight Limited.
 *
 * Sunxi adpater port is Copyright (C) 2016 by Joachim Damm
 * based on IMX adpater port Copyright (C) 2013 by Stephan Rafin
 * 
 * You can redistribute this file and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *
 */

#if defined(HAVE_SUNXI_API)

#include <p8-platform/threads/mutex.h>
#include <p8-platform/threads/threads.h>
#include <p8-platform/sockets/socket.h>
#include "adapter/AdapterCommunication.h"
#include <map>

#define SUNXI_ADAPTER_VID 0x0717 /*FIXME TBD*/
#define SUNXI_ADAPTER_PID 0x1707



namespace P8PLATFORM
{
  class CCDevSocket;
};


namespace CEC
{
  class CAdapterMessageQueueEntry;

  class CSunxiCECAdapterCommunication : public IAdapterCommunication, public P8PLATFORM::CThread
  {
  public:
    /*!
     * @brief Create a new USB-CEC communication handler.
     * @param callback The callback to use for incoming CEC commands.
     */
    CSunxiCECAdapterCommunication(IAdapterCommunicationCallback *callback);
    virtual ~CSunxiCECAdapterCommunication(void);

    /** @name IAdapterCommunication implementation */
    ///{
    bool Open(uint32_t iTimeoutMs = CEC_DEFAULT_CONNECT_TIMEOUT, bool bSkipChecks = false, bool bStartListening = true) override;
    void Close(void) override;
    bool IsOpen(void) override;
    std::string GetError(void) const;
    cec_adapter_message_state Write(const cec_command &data, bool &bRetry, uint8_t iLineTimeout, bool bIsReply) override;

    bool SetLineTimeout(uint8_t UNUSED(iTimeout)) override { return true; }
    bool StartBootloader(void) override { return false; }
    bool SetLogicalAddresses(const cec_logical_addresses &addresses) override;
    cec_logical_addresses GetLogicalAddresses(void) const override;
    bool PingAdapter(void) override { return IsInitialised(); }
    uint16_t GetFirmwareVersion(void) override;
    uint32_t GetFirmwareBuildDate(void) override { return 0; }
    bool IsRunningLatestFirmware(void) override { return true; }
    bool PersistConfiguration(const libcec_configuration & UNUSED(configuration)) override { return false; }
    bool GetConfiguration(libcec_configuration & UNUSED(configuration)) override { return false; }
    std::string GetPortName(void) override { return std::string("Sunxi"); }
    uint16_t GetPhysicalAddress(void) override;
    bool SetControlledMode(bool UNUSED(controlled)) override { return true; }
    cec_vendor_id GetVendorId(void) override;
    bool SupportsSourceLogicalAddress(const cec_logical_address address) override { return address > CECDEVICE_TV && address <= CECDEVICE_BROADCAST; }
    cec_adapter_type GetAdapterType(void) override { return ADAPTERTYPE_SUNXI; }
    uint16_t GetAdapterVendorId(void) const override { return SUNXI_ADAPTER_VID; }
    uint16_t GetAdapterProductId(void) const override { return SUNXI_ADAPTER_PID; }
    void SetActiveSource(bool UNUSED(bSetTo), bool UNUSED(bClientUnregistered)) override {}
    //bool SetAutoMode(bool automode) override { (void)automode; return false; };
    //bool GetStats(struct cec_adapter_stats* stats) override { (void)stats; return false; };

    ///}

    /** @name PLATFORM::CThread implementation */
    ///{
    void *Process(void);
    ///}

  private:
    bool IsInitialised(void) const { return m_dev != 0; };

    std::string                 m_strError; /**< current error message */

    //cec_logical_addresses       m_logicalAddresses;
    cec_logical_address         m_logicalAddress;

    P8PLATFORM::CMutex            m_mutex;
    P8PLATFORM::CCDevSocket       *m_dev;	/**< the device connection */
    
    P8PLATFORM::CMutex            m_messageMutex;
    uint32_t                    m_iNextMessage;
    std::map<uint32_t, CAdapterMessageQueueEntry *> m_messages;
  };
  
};

#endif
