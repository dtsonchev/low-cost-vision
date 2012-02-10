#include <modbus/modbus.h>
#include <assert.h>
#include <stdlib.h>
#include "config.h"

int main(void)
{
    modbus_t* ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1);     
    assert(ctx != NULL);
  
    struct timeval timeout_end;
    struct timeval timeout_begin;
    modbus_get_byte_timeout(ctx, &timeout_end);
    timeout_end.tv_usec = MODBUS_TIMEOUT_END;
    modbus_set_byte_timeout(ctx, &timeout_end);

    modbus_get_response_timeout(ctx, &timeout_begin);
    timeout_begin.tv_usec = MODBUS_TIMEOUT_BEGIN;
    modbus_set_response_timeout(ctx, &timeout_begin);

    assert(modbus_connect(ctx) != -1);

    modbus_set_slave(ctx, 0);
    
    usleep(8000);
    modbus_write_register(ctx, 0x001E, 0x0000);
    usleep(8000);
    
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
