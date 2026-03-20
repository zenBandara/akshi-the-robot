import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import socket

cred = credentials.Certificate("serviceAccountKey.json")

firebase_admin.initialize_app(cred, {
    "databaseURL": "https://akshi-robot-default-rtdb.firebaseio.com"
})



# ✅ SET (update IP)
def update_connected_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()

        db.reference("connected_ip").set(ip)
        print("Connected IP updated:", ip)

    except Exception as e:
        print("Error updating IP:", e)

