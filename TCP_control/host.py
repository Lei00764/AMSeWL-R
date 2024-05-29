import socket
import subprocess
import psutil
import time
import logging
import errno


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(),
        # Replace with the desired path
        logging.FileHandler("/home/parallels/Desktop/logfile.log"),
    ],
)


def bind_socket(port):
    """Bind the socket to the specified port.

    Args:
        port (int): The port number.

    Returns:
        socket.socket: The bound socket object.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("", port))
        logging.info(f"Socket bound to port {port}.")
    except OSError as e:
        logging.error(f"Error binding socket to port {port}: {e}")
        if e.errno == errno.EADDRINUSE:
            logging.info(f"Attempting to release port {port}.")
            release_port(port)
            # Retry binding after releasing the port
            try:
                s.bind(("", port))
                logging.info(f"Socket bound to port {port}.")
            except OSError as e:
                logging.error(
                    f"Failed to bind socket to port {port} after releasing: {e}"
                )
                raise
        else:
            raise
    return s


def release_port(port):
    """Release the port by killing the process using the port.

    Args:
        port (int): The port number.
    """
    for process in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            if isinstance(process.info["cmdline"], list):
                if f":{port}" in process.info["cmdline"]:
                    pid = process.info["pid"]
                    logging.info(
                        f"Found process using port {port} with PID {pid}. Killing the process..."
                    )
                    psutil.Process(pid).terminate()
                    logging.info(f"Process with PID {pid} killed.")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            logging.error(
                "Error occurred while trying to kill the process using the port."
            )


def kill_roslaunch():
    """Kill the roslaunch process."""
    command_line = "roslaunch"
    for process in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            if isinstance(process.info["cmdline"], list):
                process_cmdline = " ".join(process.info["cmdline"])
                if command_line in process_cmdline:
                    pid = process.info["pid"]
                    logging.info(
                        f"Found process with command line '{command_line}' running with PID {pid}. Killing the process..."
                    )
                    psutil.Process(pid).terminate()
                    logging.info(f"Process with PID {pid} killed.")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            logging.error("Error occurred while trying to kill the process.")


def start_play_script(script_path):
    """Start a data bag play script.

    Args:
        script_path (str): The path to the play script.

    Returns:
        subprocess.Popen: The subprocess object representing the running process.
    """
    try:
        process = subprocess.Popen([script_path])
        logging.info(f"Started play script '{script_path}' with PID {process.pid}.")
        return process
    except subprocess.CalledProcessError as e:
        logging.error(f"Error starting play script '{script_path}': {e}")
        return None


if __name__ == "__main__":
    target_port = 6543

    # Bind the socket to the specified port
    s = bind_socket(target_port)

    s.listen()

    while True:
        client, addr = s.accept()
        logging.info(f"Connected by {addr}")

        while True:
            data = client.recv(1024)
            if not data:
                break

            message = data.decode("utf-8")
            logging.info(f"Received: {message}")

            if message == "kill":
                kill_roslaunch()
            elif message == "switch 2d":
                kill_roslaunch()
                time.sleep(1)
                start_play_script("/home/parallels/Desktop/play_2D.sh")
            elif message == "switch 3d":
                kill_roslaunch()
                time.sleep(1)
                start_play_script("/home/parallels/Desktop/play_3D.sh")
            elif message == "switch 2d real time":
                kill_roslaunch()
                time.sleep(1)
                start_play_script("/home/parallels/Desktop/play_2D_real_time.sh")

        client.close()

    s.close()
