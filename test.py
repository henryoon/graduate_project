data = 18000

bd = data.to_bytes(2, "big")

print(type(bd[0]))
print(bd[1])


ba = bytearray(2)

ba[0] = bd[0]
import serial
import threading
import time


def create_packet(p: float, w: float):
    H1 = 255
    H2 = 254
    ID = 0
    DS = 7
    MD = 1
    DI = 0
    P = int(p / 0.01)  # DEG
    W = int(w / 0.1)  # RPM
    CS = ~(ID + DS + MD + DI + P + W) & 0xFF

    data = bytearray(6)

    data[0] = H1  # Header
    data[1] = H2  # Header
    data[2] = ID  # ID
    data[3] = DS  # Data size
    data[4] = CS  # Checksum
    data[5] = MD  # Mode
    data[6]  # Direction (CW, CCW)
    data[7]  # Position
    data[8]  # Position
    data[9]  # Velocity
    data[10]  # Velocity
    return data


def receive_data(ser: serial.Serial):
    while True:
        data = ser.read(6)
        print(data)

        continue
        data = ser.read()
        print(data)
        idx = 0
        for i in data:
            print(i)
            idx += 1

            if idx == 6:
                idx = 0
                break


def test(ser: serial.Serial):
    packet = create_packet()
    print(packet, len(packet))
    # ser.write(packet)
    time.sleep(0.1)
    response = ser.read()
    # response = ser.read(12)
    print(response)


# if __name__ == "__main__":
#     # 시리얼 포트 설정 (포트 이름과 보레이트를 실제 환경에 맞게 수정)
#     ser = serial.Serial("/dev/ttyUSB1", baudrate=9600, timeout=1)

#     thread = threading.Thread(target=receive_data, args=(ser,))
#     thread.daemon = True
#     thread.start()

#     while True:
#         ser.write(create_packet())
#         time.sleep(0.1)
#         # test(ser)
