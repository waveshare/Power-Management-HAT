# Brief Introduction

This project only used for Power-Management-HAT, a brife guide to tell you how to deploy in Windows arduino IDE and Raspberry,hope you enjoy it.

# Deploy in Windows
1. Download the PowerManagementHAT folder by git command to you computer(Windows) which has installed Arduino IDE.

2. Copy the folder "PowerManagementHAT" to  hardware folder in the Arduino IDE install Path.

3. Please note that the Path should just like this:

  ``
  D:\Program Files (x86)\Arduino\hardware\PowerManagementHAT\PowerManagementHAT\...
  ``


# Deploy in Rapsberry


1. Firstly, turn on you raspberry,open the command console.

2. Secondly, Download the shell Script:

```sh
wget https://raw.githubusercontent.com/waveshare/Power-Management-HAT/master/Power-Management-HAT-Setup.sh
```

3. Then, Make it executable by: 
```sh
chmod a+x Power-Management-HAT-Setup.sh
```

4. Next, Execute the script with sudo 
```sh
sudo ./Power-Management-HAT-Setup.sh
```

5. Finally, Reboot

Or you can just do this:

Copy and paste the following code to command console.

```sh
cd /usr/src
wget https://raw.githubusercontent.com/waveshare/Power-Management-HAT/master/Power-Management-HAT-Setup.sh
chmod a+x Power-Management-HAT-Setup.sh
sudo ./Power-Management-HAT-Setup.sh
exit
```
Then press "Enter" to execute those commander.
