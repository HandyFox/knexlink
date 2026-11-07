"""Linkage Designer - start here:   python main.py   [optional design file]"""
import os
import sys

os.environ.setdefault("QT_API", "pyside6")

from qtpy import QtWidgets  # noqa: E402

from linkage.model import Project  # noqa: E402
from linkage.ui import MainWindow  # noqa: E402


def _excepthook(kind, value, tb):
    import traceback
    text = "".join(traceback.format_exception(kind, value, tb))
    sys.__stderr__.write(text)
    try:
        QtWidgets.QMessageBox.warning(None, "Linkage Designer - something went wrong",
                                      "That action hit an error (the app is still running):\n\n"
                                      + text[-1500:])
    except Exception:
        pass


def main():
    sys.excepthook = _excepthook
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Linkage Designer")
    app.setStyle("Fusion")
    project, path = None, None
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        with open(sys.argv[1], "r", encoding="utf-8") as fh:
            project = Project.from_json(fh.read())
        path = sys.argv[1]
    win = MainWindow(project, path)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
