source /opt/tbotspython/bin/activate

pip freeze | xargs sudo pip uninstall -y
sudo pip install -r environment_setup/macos_requirements.txt
