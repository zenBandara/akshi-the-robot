import sys
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel
from PySide6.QtCore import Qt, QTimer


class CalibrationWindow(QMainWindow):
    def __init__(self, server):
        super().__init__()

        self.server = server

        # Full screen
        self.showFullScreen()
        self.hide()

        # Default background
        self.setStyleSheet("background-color: #87CEEB;")

        # Label
        self.label = QLabel("Not started yet", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 36px; font-weight: bold; color: black;")

        self.setCentralWidget(self.label)

        # Store last status to avoid flicker
        self.last_status = ""

        # Timer to update UI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_from_server)
        self.timer.start(50)  # faster update

    def update_from_server(self):
        data = self.server.last_client_data

        msg_type = None
        if data:
            msg_type = data.get("type")

        if msg_type == "start_session":
            self.show()

        if msg_type == "end_session":
            self.hide()
            self.label.setText("Not started yet")
            self.last_status = ""
            return
        # No data case
        if not data:
            if self.last_status != "Not started yet":
                self.label.setText("Not started yet")
                self.setStyleSheet("background-color: #87CEEB;")
                self.last_status = "Not started yet"
            return

        status = data.get("calibration_status", "Not started yet")

        if not status:
            status = "Not started yet"

        # Update only if changed
        if status != self.last_status:
            self.label.setText(status)

            # Change background color based on status
            if status[:14] == "Keep eyes OPEN":
                self.setStyleSheet("background-color: lightgreen;")
            elif status[:16] == "Keep eyes CLOSED":
                self.setStyleSheet("background-color: orange;")
            elif status[:4] == "DONE":
                self.setStyleSheet("background-color: lightgray;")
            else:
                self.setStyleSheet("background-color: #87CEEB;")

            self.last_status = status

            print("UI updated:", status)  # optional debug

        # Close when DONE
        if status.startswith("DONE"):
            self.hide()

    # ESC to close manually
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


# Do NOT run this file directly
if __name__ == "__main__":
    print("Run this UI from main file")