packer {
  required_plugins {
    git = {
      version = ">=v0.3.2"
      source  = "github.com/ethanmdavidson/git"
    }
  }
}

# data "git-commit" "cwd-head" {}

locals {
  # git_sha = data.git-commit.cwd-head.hash
}

source "arm" "raspbian" {
  file_urls             = ["https://downloads.raspberrypi.com/raspios_arm64/images/raspios_arm64-2024-07-04/2024-07-04-raspios-bookworm-arm64.img.xz"]
  file_checksum_url     = "https://downloads.raspberrypi.com/raspios_arm64/images/raspios_arm64-2024-07-04/2024-07-04-raspios-bookworm-arm64.img.xz.sha256"
  file_checksum_type    = "sha256"
  file_target_extension = "xz"
  file_unarchive_cmd    = ["xz", "--decompress", "$ARCHIVE_PATH"]
  image_build_method    = "resize"
  image_path            = "pupOS_pios.img"
  image_size            = "16G"
  image_type            = "dos"
  image_partitions {
    name         = "boot"
    type         = "c"
    start_sector = "2048"
    filesystem   = "fat"
    size         = "256M"
    mountpoint   = "/boot/firmware"
  }
  image_partitions {
    name         = "root"
    type         = "83"
    start_sector = "526336"
    filesystem   = "ext4"
    size         = "0"
    mountpoint   = "/"
  }
  image_chroot_env             = ["PATH=/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin"]
  qemu_binary_source_path      = "/usr/bin/qemu-aarch64-static"
  qemu_binary_destination_path = "/usr/bin/qemu-aarch64-static"
}

build {
  sources = ["source.arm.raspbian"]

  provisioner "shell" {
    inline = ["sleep 10"]
  }

# For some reason would never complete
#   provisioner "shell" {
#     inline = [
#      "while [ ! -f /var/lib/cloud/instance/boot-finished ]; do echo 'Waiting for cloud-init...'; sleep 1; done"
#     ]
#   }

  # Required to get internet access
  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf /etc/resolv.conf.bk",
      "sudo echo 'nameserver 8.8.8.8' > /etc/resolv.conf",
    ]
  }

  # Set hostname to 'pupper'
  provisioner "shell" {
    script = "set_hostname.sh"
  }



#   provisioner "file" {
#     source      = "user-data"
#     destination = "/boot/firmware/user-data"
#   }

  provisioner "shell" {
    script = "provision.sh"
  }

  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf.bk /etc/resolv.conf",
    ]
  }
}
