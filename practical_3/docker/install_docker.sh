#!/bin/bash

# Docker installation according to https://docs.docker.com/engine/install/ubuntu/
echo "Docker not found, installing..."
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
echo "Docker installed"
               
# Docker post-installation according to https://docs.docker.com/engine/install/linux-postinstall/
echo "Running docker post install..."

# Create docker group if not already done
if ! [ $(getent group docker) ] ; then
    sudo groupadd docker
fi

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker


# If nvidia, install nividia-docker2

# # nvidia-docker2 install according to: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
# echo "Installing nvidia-docker2"
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
#     && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#     && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
#     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
# sudo apt-get update
# sudo apt-get install -y nvidia-docker2
# sudo systemctl restart docker
# echo "Installation done."



