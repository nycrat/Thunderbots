import avah_test
from pyqtgraph.Qt.QtWidgets import (
    QAbstractItemView,
    QDialog,
    QListWidget,
    QPushButton,
    QVBoxLayout,
)


class GLTestSelectorDialog(QDialog):
    def __init__(self, parent, tests):
        super().__init__(parent)

        self.setWindowTitle("Run Tests")
        self.setModal(True)
        self.setMinimumWidth(400)

        run_button = QPushButton("Run")
        run_button.clicked.connect(self.__run_test)

        runtime_select_list = QListWidget()
        runtime_select_list.setSelectionMode(
            QAbstractItemView.SelectionMode.MultiSelection
        )
        runtime_select_list.setFixedHeight(200)
        runtime_select_list.addItems(tests)

        self.runtime_select_list = runtime_select_list

        layout = QVBoxLayout(self)
        layout.addWidget(runtime_select_list)
        layout.addWidget(run_button)

    def __run_test() -> None:
        avah_test.other_function()
