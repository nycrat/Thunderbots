import os
from typing import Callable, override

import GLTestSelectorDialog
import qtawesome as qta
from pyqtgraph.Qt import QtWidgets

from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar


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
        if not hasattr(self, "test_selector_dialog"):
            self.test_selector_dialog = GLTestSelectorDialog(self.parent(), ["woah", "other"])
        self.test_selector_dialog.show()

        # cwd = os.getenv("WORKSPACE_DIR")
        # def print_tests(tests):
        #     proc = subprocess.Popen(
        #         ["bazel", "query", "kind(py_test, //...)"],
        #         stdout=subprocess.PIPE,
        #         stderr=subprocess.STDOUT,
        #         text=True,
        #         cwd=cwd,
        #     )
        #
        #     for line in proc.stdout:
        #         if line.startswith("//"):
        #             tests.append(line.strip())
        #
        # tests = []
        #
        # thread = threading.Thread(
        #         target=print_tests,
        #         args=(tests,))
        #
        # thread.start()
        # thread.join()
        #
        # print(tests)
