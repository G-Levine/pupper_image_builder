#!/bin/bash -e

# Function to retry a command
retry_command() {
  local cmd="$1"
  local max_attempts="$2"
  local attempt=0

  # Loop to retry the command
  while [ $attempt -lt $max_attempts ]; do
    echo "Attempting to run: $cmd (Attempt $((attempt + 1))/$max_attempts)"
    
    # Run the command
    if eval "$cmd"; then
      echo "Command succeeded!"
      return 0  # Exit the function as success
    else
      attempt=$((attempt + 1))
      echo "Attempt $attempt/$max_attempts failed. Retrying in 5 seconds..."
      sleep 1
    fi
  done

  # If max attempts reached
  echo "Command failed after $max_attempts attempts."
  return 1  # Indicate failure
}

# TODO UPDATE THE KERNEL VERSION IF CHANGES
sudo apt install -y linux-headers-raspi build-essential

retry_command "git clone https://github.com/hailo-ai/hailort-drivers.git --branch v4.17.0" 20
cd hailort-drivers/

cd linux/pcie

# TODO UPDATE THE KERNEL VERSION IF CHANGES
# Could copy from the raspberry pi?
# FAILS HERE
make kernelver=6.8.0-1012-raspi all
sudo make kernelver=6.8.0-1012-raspi install
cd ../..
./download_firmware.sh
sudo mkdir -p /lib/firmware/hailo
sudo mv hailo8_fw.4.17.0.bin /lib/firmware/hailo/hailo8_fw.bin
sudo cp ./linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/

retry_command "wget http://archive.raspberrypi.com/debian/pool/main/h/hailort/hailort_4.17.0_arm64.deb" 20
sudo apt install -y ./hailort_4.17.0_arm64.deb

# Requires systemd to install?
retry_command "wget http://archive.raspberrypi.com/debian/pool/main/h/hailo-tappas-core-3.28.2/hailo-tappas-core-3.28.2_3.28.2_arm64.deb" 20
sudo apt install -y ./hailo-tappas-core-3.28.2_3.28.2_arm64.deb

sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install -y python3.11-full python3.11-dev

sudo update-alternatives --install /usr/local/bin/python3 python3 /usr/bin/python3.11 3

sudo apt install -y meson

cd /home/pi
git clone https://github.com/hailo-ai/hailo-rpi5-examples.git
cd hailo-rpi5-examples
source setup_env.sh
pip install -r requirements.txt
./download_resources.sh
./compile_postprocess.sh

pip install --ignore-installed PyGObject
pip install numpy --upgrade