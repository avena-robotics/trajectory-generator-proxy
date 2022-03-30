import modbus_server
import rs485_com
import asyncio


async def main():
    rs_com = rs485_com.RSComm()
    mb_serv = modbus_server.ModbusServer()

    t1=asyncio.create_task(rs_com.update_stm_status(mb_serv.jtc_status))
    t2=asyncio.create_task(mb_serv.handle_request())

    g = await asyncio.gather(t1,t2)


a = asyncio.run(main())