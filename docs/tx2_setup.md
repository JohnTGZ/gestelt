# Hardware 
1. Board: https://auvidea.eu/product/j120a-70719/
2. Developer guide: https://developer.download.nvidia.com/assets/embedded/secure/jetson/TX2/docs/nv_jetson_tx2_developer_kit_user_guide.pdf?cTEmTbOCZYS53c-A1k-ap7S5O85JnJyxnwq_-0sFOhc9728D2oGJDZBwnp4dqxM8iXIqTniXcnP88nXjfS2NFwDfd_mt005ERQmfof_ym26zOopVm77x-TBlojgzyhl70dgwuPyFONgfOj8lGXsICSS0zmKFhfx6xmPZ3EEKxVTI51Q_ycvyb6XMWabUbamc&t=eyJscyI6ImdzZW8iLCJsc2QiOiJodHRwczovL3d3dy5nb29nbGUuY29tLyJ9

3. Comparison between TX2, Xavier and Nano: 
https://3dvisionlabs.com/2020/05/22/jetson-xavier-nx-compared-to-jetson-tx2-and-jetson-nano/



# Compatible versions:
1. JetPack 4.6.4 [https://developer.nvidia.com/jetpack-sdk-464#collapseAllJetson]
2. Jetson Linux 32.7.4 [https://developer.nvidia.com/embedded/linux-tegra-r3274]
3. Jetpack SDK 4.6.4 (https://developer.nvidia.com/jetpack-sdk-464)


# Steps:

1. On host:
    - `sudo apt-get install qemu-user-static`
    - Install Jetson SDK Manager (https://developer.nvidia.com/sdk-manager)
2. Put board into force recovery mode (RCM) and power on
3. Connect board (Micro USB-B) to computer (USB-A)
4. If you are running Ubuntu 20.04 and above, you need to "spoof" your Ubuntu version as the Jetpack SDK for TX2 is only available on Ubuntu 18.04 host and below. [Reference](https://forums.developer.nvidia.com/t/sdkmanager-not-supported-on-linux/71742/4)
    - `sudo vim /usr/lib/os-release` and put in the following:
    -   ```bash
            NAME="Ubuntu"
            VERSION="18.04 (Bionic Beaver)"
            ID=ubuntu
            ID_LIKE=debian
            PRETTY_NAME="Ubuntu 18.04"
            VERSION_ID="18.04"
            VERSION_CODENAME=bionic
            UBUNTU_CODENAME=bionic
        ```
    - `export LSB_OS_RELEASE=/usr/lib/os-release-bionic && sdkmanager`
5. Using SDKManager, you should be able to download and flash JetPack 4.6.4


# References
1. https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3274/index.html?_gl=1*xsiprb*_gcl_au*NDMwMjMwNTczLjE3MTA5Mjg0Mjg.


# Others
```bash
NAME="Ubuntu"
VERSION="20.04.6 LTS (Focal Fossa)"
ID=ubuntu
ID_LIKE=debian
PRETTY_NAME="Ubuntu 20.04.6 LTS"
VERSION_ID="20.04"
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
VERSION_CODENAME=focal
UBUNTU_CODENAME=focal
```
