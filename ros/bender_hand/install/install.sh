#!/bin/bash
#
# Run me like this
# > bash install.sh
#
# DO NOT USE ONE OF THIS:
# > source install.sh
# > . install.sh
# > ./install.sh
#

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #
source "$UCHILE_WS"/system/install/util/pkg_install.bash

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# install hand rules
uchile_cd bender_hand
sudo cp -f install/bender_hand.rules /etc/udev/rules.d/bender_hand.rules

# Permisos para lectura de puertos
var_group=$(groups "$USER" | grep -o -w -c dialout)
if [ "$var_group" = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\nPort permissions\n"
  sudo usermod -a -G dialout "$USER"

  echo "The computer must be restarted in order to complete the installation"
  echo -e "... Do you want to reboot the pc now?\n"
  echo "(y)es (n)o:"
  read var_reboot

  if [ "$var_reboot" = "y" ]; then

    echo -e "\nDone.\n ;)\n"
    sudo shutdown -r now

  else
    echo "... Remember to reboot the computer!!!..."
    echo -e "\nDone.\n :)\n"
  fi 
fi

echo -e "\nDone.\n :)\n"
# :)
