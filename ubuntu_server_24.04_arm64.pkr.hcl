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

source "arm" "ubuntu" {
  file_urls             = ["https://cdimage.ubuntu.com/releases/24.04.1/release/ubuntu-24.04.1-preinstalled-server-arm64+raspi.img.xz"]
  file_checksum_url     = "https://cdimage.ubuntu.com/releases/24.04.1/release/SHA256SUMS"
  file_checksum_type    = "sha256"
  file_target_extension = "xz"
  file_unarchive_cmd    = ["xz", "--decompress", "$ARCHIVE_PATH"]
  image_build_method    = "resize"
  image_path            = "pupOS_ubuntu_light_desktop.img"
  image_size            = "12G"
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
  sources = ["source.arm.ubuntu"]

  provisioner "file" {
    source      = "user-data"
    destination = "/boot/firmware/user-data"
  }

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

  # Fix ubuntu sources
  provisioner "shell" {
    script = "fix_ubuntu_sources.sh"
  }

  provisioner "shell" {
    script = "provision.sh"
  }

  provisioner "shell" {
    inline = [
      "sudo mv /etc/resolv.conf.bk /etc/resolv.conf",
    ]
  }
}
