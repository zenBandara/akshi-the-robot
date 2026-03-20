import os
import sys
import json
import subprocess
import threading
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_PATH = os.path.join(BASE_DIR, "main.py")
ANIMATION_FILE = os.path.join(BASE_DIR, "animation.json")
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

def ensure_pywebview():
    try:
        import webview
        return True
    except Exception:
        try:
            subprocess.run([sys.executable, "-m", "pip", "install", "pywebview"], check=False)
            import webview
            return True
        except Exception:
            return False

def build_html():
    with open(ANIMATION_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    anim_json = json.dumps(data)
    return (
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>Opening...</title>"
        "<style>html,body{margin:0;height:100%;background:#000}#lottie{width:100vw;height:100vh}</style>"
        "</head><body><div id=\"lottie\"></div>"
        "<script src=\"https://cdnjs.cloudflare.com/ajax/libs/bodymovin/5.12.2/lottie.min.js\"></script>"
        f"<script>var animData={anim_json};"
        "lottie.loadAnimation({container:document.getElementById('lottie'),renderer:'svg',loop:true,autoplay:true,animationData:animData});" 
        "</script></body></html>"
    )

def start_serial(window):
    try:
        import serial
    except Exception:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except Exception:
        ser = None
    def loop():
        main_started = False
        try:
            while True:
                if ser is None:
                    time.sleep(1)
                    continue
                raw = ser.readline().decode(errors='ignore').strip()
                if not raw:
                    continue
                if raw and raw[0].isdigit() and ',' in raw:
                    raw = raw.split(',', 1)[1].strip()
                parts = raw.split(',')
                vals = {}
                for p in parts:
                    p = p.strip()
                    if '=' in p:
                        k, v = p.split('=', 1)
                        k = k.strip(); v = v.strip()
                        try:
                            vals[k] = int(v)
                        except Exception:
                            vals[k] = None
                s1 = vals.get("S1")
                if s1 is not None and s1 <= 15 and not main_started:
                    try:
                        subprocess.Popen(["python3", "main.py"], cwd=BASE_DIR)
                    finally:
                        main_started = True
                        try:
                            window.destroy()
                        except Exception:
                            pass
                        break
        finally:
            try:
                if ser is not None:
                    ser.close()
            except Exception:
                pass
    threading.Thread(target=loop, daemon=True).start()

def run():
    ok = ensure_pywebview()
    if not ok:
        return
    import webview
    html = build_html()
    window = webview.create_window("Opening...", html=html, fullscreen=True)
    start_serial(window)
    webview.start()

if __name__ == "__main__":
    run()