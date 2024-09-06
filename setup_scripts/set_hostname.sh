# Set hostname to 'pupper'
CURRENT_HOSTNAME=`hostname`
hostnamectl set-hostname pupper
sed -i "s/127.0.1.1.*$CURRENT_HOSTNAME/127.0.1.1\tpupper/g" /etc/hosts
