import socket

HOST = "192.168.0.184"
PORT = 2026

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"Hello AI Server\n")
    print("메시지 전송 완료")
