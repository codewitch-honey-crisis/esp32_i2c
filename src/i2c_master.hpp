#ifndef HTCW_ESP32_I2C_MASTER_HPP
#define HTCW_ESP32_I2C_MASTER_HPP
#include <atomic>
#include <inttypes.h>
#include <stdio.h>
#include "driver/i2c.h"
#include "soc/gpio_sig_map.h"

#define MATRIX_DETACH_OUT_SIG 0x100
#define MATRIX_DETACH_IN_LOW_PIN 0x30
#define MATRIX_DETACH_IN_LOW_HIGH 0x38

namespace esp32 {
    class i2c_master;
    class i2c_master_command;
    class i2c final {
        friend class i2c_master;
        friend class i2c_master_command;
    private:
        static std::atomic<esp_err_t> m_last_error;
        i2c()=delete;
        i2c(const i2c& rhs)=delete;
        i2c& operator=(const i2c& rhs)=delete;
        i2c(i2c&& rhs)=delete;
        i2c& operator=(i2c&& rhs)=delete;
        ~i2c()=delete;
        static void last_error(esp_err_t last_error) {
            m_last_error=last_error;
        }
    public:
        static esp_err_t last_error() { return m_last_error;}
    };
    std::atomic<esp_err_t> i2c::m_last_error={ESP_OK};
    
    class i2c_master_command final {
        i2c_cmd_handle_t m_handle;
        i2c_master_command(const i2c_master_command& rhs)=delete;
        i2c_master_command& operator=(const i2c_master_command& rhs)=delete;
    public:
        i2c_master_command() : m_handle(nullptr) {
            m_handle = i2c_cmd_link_create();
        }
        i2c_master_command(i2c_master_command&& rhs) : m_handle(rhs.m_handle) {
            rhs.m_handle=nullptr;
        }
        i2c_master_command& operator=(i2c_master_command&& rhs)  {
            m_handle=rhs.m_handle;
            rhs.m_handle=nullptr;
            return *this;
        }
        ~i2c_master_command() {
            if(m_handle)
                i2c_cmd_link_delete(m_handle);
        }
        i2c_cmd_handle_t handle() const {
            return m_handle;
        }
        bool initialized() const {
            return nullptr!=m_handle;
        }
        bool start() {
            esp_err_t res = i2c_master_start(m_handle);
            if(ESP_OK!=res) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }
        bool stop() {
            esp_err_t res = i2c_master_stop(m_handle);
            if(ESP_OK!=res) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }
        bool read(void* data,size_t size,i2c_ack_type_t ack=i2c_ack_type_t::I2C_MASTER_ACK) {
            if(nullptr==data) {
                i2c::last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            esp_err_t res = i2c_master_read(m_handle,(uint8_t*)data,size,ack);
            if(ESP_OK!=res) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }
        bool read(uint8_t* data,i2c_ack_type_t ack=i2c_ack_type_t::I2C_MASTER_ACK) {
            if(nullptr==data) {
                i2c::last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            esp_err_t res = i2c_master_read_byte(m_handle,data,ack);
            if(ESP_OK!=res) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }
        bool write(const void* data,size_t size,bool ack=false) {
            if(nullptr==data) {
                i2c::last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            esp_err_t res = i2c_master_write(m_handle,(uint8_t*)data,size,ack);
            if(res!=ESP_OK) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }
        bool write(uint8_t data,bool ack=false) {
            esp_err_t res = i2c_master_write_byte(m_handle,data,ack);
            if(res!=ESP_OK) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }
        bool begin_read(uint8_t address,bool ack=false) {
            if(!write(address<<1|I2C_MASTER_READ,ack))
                return false;
            return true;
        }
        bool begin_write(uint8_t address,bool ack=false) {
           if(!write(address<<1|I2C_MASTER_WRITE,ack))
                return false;
            return true;
        }
        bool read_register(uint8_t address,uint8_t regist, void* result,size_t size) {
            if(nullptr==result || 0==size) {
                i2c::last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            if(!begin_write(address,true)) 
                return false;
            if(!write(regist,true))
                return false;
            if(!start())
                return false;
            if(!begin_read(address,true))
                return false;
            if (size > 1) {
                if(!read(result, size - 1, I2C_MASTER_ACK))
                    return false;
            }
            if(!read(((uint8_t*)result + size - 1), I2C_MASTER_NACK))
                return false;
            if(!stop())
                return false;
            return true;
        }
        bool write_register(uint8_t address,uint8_t regist, const void* data,size_t size) {
            if(nullptr==data || 0==size) {
                i2c::last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            if(!begin_write(address,true)) 
                return false;
            if(!write(regist,true))
                return false;
            if(!write(data,size,true))
                return false;
            if(!stop())
                return false;
            return true;
        }
    };
    class i2c_master final {
        bool m_initialized;
        i2c_port_t m_port;
        i2c_config_t m_configuration;
        i2c_master(const i2c_master& rhs)=delete;
        i2c_master& operator=(const i2c_master& rhs)=delete;
    public:
        i2c_master(
            i2c_port_t i2c_port=I2C_NUM_0, 
            gpio_num_t sda=GPIO_NUM_21,
            gpio_num_t scl=GPIO_NUM_22,
            bool sda_pullup=true,
            bool scl_pullup=true, 
            uint32_t frequency=100000,
            int interrupt_flags=0
            ) : m_initialized(false) {
            m_configuration.mode=i2c_mode_t::I2C_MODE_MASTER;
            m_configuration.sda_io_num = (int)sda;
            m_configuration.scl_io_num = (int)scl;
            m_configuration.sda_pullup_en = sda_pullup;
            m_configuration.scl_pullup_en = scl_pullup;
            m_configuration.master.clk_speed=frequency;
            esp_err_t res = i2c_param_config(i2c_port, &m_configuration);
            if (res != ESP_OK) {
                i2c::last_error(res);
                return;
            }
            res = i2c_driver_install(i2c_port, m_configuration.mode, 0, 0, interrupt_flags);
            if (res != ESP_OK) {
                i2c::last_error(res);
                return;
            } 
            m_port=i2c_port;
            m_initialized=true;
        }
        i2c_master(i2c_master&& rhs) : m_initialized(rhs.m_initialized),m_configuration(rhs.m_configuration) {
            rhs.m_initialized=false;
        }
        i2c_master& operator=(i2c_master&& rhs) {
            m_initialized=rhs.m_initialized;
            m_configuration = rhs.m_configuration;
            rhs.m_initialized=false;
            return *this;
        }
        ~i2c_master() {m_initialized=false;}
        bool initialized() const {
            return m_initialized;
        }
        bool execute(const i2c_master_command& command,TickType_t timeoutTicks=portMUX_NO_TIMEOUT) {
            esp_err_t res = i2c_master_cmd_begin(m_port,command.handle(),timeoutTicks);
            if(ESP_OK!=res) {
                i2c::last_error(res);
                return false;
            }
            return true;
        }

    };
}
#endif