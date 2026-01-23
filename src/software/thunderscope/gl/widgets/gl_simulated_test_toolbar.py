from pyqtgraph.Qt import QtWidgets
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
import qtawesome as qta
from typing import override, Callable
import subprocess
import threading


class GLSimulatedTestToolbar(GLToolbar):
    """A toolbar with controls to run simulated tests within Thunderscope"""

    def __init__(self, parent: QtWidgets.QWidget, start: Callable[[], None]):
        """Initializes the toolbar and constructs its layout

        :param parent: the parent to overlay this toolbar over
        """
        super(GLSimulatedTestToolbar, self).__init__(parent=parent)

        self.run_test_button = self.setup_icon_button(
            qta.icon("fa5s.play"),
            "Runs simluated test",
            self.__run_test,
        )

        self.start = start

        self.layout().addWidget(QtWidgets.QLabel("<b>Simulated Tests</b>"))
        self.add_separator(self.layout())
        self.layout().addWidget(self.run_test_button)

    @override
    def refresh(self) -> None:
        """Refreshes the UI to update toolbar position"""
        self.move(0, self.parentWidget().geometry().bottom() - self.height())

    def __run_test(self):
        print("RUN TEST")
        # self.start()
        bazel_target = (
            "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic_test"
        )

        def run_bazel_test(target):
            proc = subprocess.Popen(
                ["bazel", "test", target, "--", "--from_thunderscope"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd="/Users/avah/2_school/team/thunderbots/src",
            )
            for line in proc.stdout:
                print(line, end="")
            proc.wait()

        thread = threading.Thread(target=run_bazel_test, args=(bazel_target,))
        thread.start()
