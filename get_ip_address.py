import subprocess

def get_ip():
    ip = subprocess.getoutput("hostname -I")
    return ip.strip()

print("Raspberry Pi IP:", get_ip())