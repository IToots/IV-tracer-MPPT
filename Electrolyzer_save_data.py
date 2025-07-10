import serial
import time
import os

def collect_data(command="START\n", port="COM17", output_dir="data_logs", filename="log.csv"):
    # Create full path
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    output_path = os.path.join(output_dir, filename)

    print(f"Opening serial port {port}...")
    print(f"Saving to: {output_path}")
    
    # Step 1: Create the file and write the header
    with open(output_path, 'w', encoding='utf-8') as file:
        file.write("Time(s),Voltage(V),Current(mA)\n")

    try:
        with serial.Serial(port, 115200, timeout=1) as conn:
            print(f"Sending command: {command.strip()}")
            conn.write(command.encode())

            while True:
                if conn.in_waiting > 0:
                    line = conn.readline().decode(errors="ignore").strip()
                    
                    if line == "END":
                        print("Received END. Done.")
                        break
                    elif line == "FAILED":
                        print("Received FAILED. Aborting.")
                        break
                    else:
                        print(f"Data: {line}")
                        # Open the file, append line, then close it
                        with open(output_path, 'a', encoding='utf-8') as file:
                            file.write(line + "\n")

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    collect_data(output_dir="C:/Users/ruiui/Desktop/", filename="session_01.csv")