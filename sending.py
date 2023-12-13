import socket
import getch

def get_char():
    # Get a single character from the user without waiting for Enter
    return getch.getch()

def main():
    # Setting ip address and port
    receiver_ip = '172.20.10.6'
    receiver_port = 4444

    # Setting UDP socket
    sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            # Input data without waiting for Enter
            data = get_char()

            if data.lower() == 'exit':
                break  # Exit the loop if the user types 'exit'

            # Send to receiver
            sender_socket.sendto(data.encode(), (receiver_ip, receiver_port))

    except KeyboardInterrupt:
        print("Sending end.")
    finally:
        # Close socket
        sender_socket.close()

if __name__ == "__main__":
    main()
