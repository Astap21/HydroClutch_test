#  coding=utf8
import sys
import os
from PyQt5 import QtWidgets, QtCore, QtGui
#import serial.tools.list_ports

from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice
from time import sleep
import traceback

import time
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
import statistics

import xlrd
import xlwt
import glob


# для отлавливания ошибок
def log_uncaught_exceptions(ex_cls, ex, tb):
    text = '{}: {}:\n'.format(ex_cls.__name__, ex)
    import traceback
    text += ''.join(traceback.format_tb(tb))

    print(text)
    QtWidgets.QMessageBox.critical(None, 'Error', text)
    # quit()


import sys

sys.excepthook = log_uncaught_exceptions
# добаление созданных виджетов
import My_1_form, Add_book_Dialog


def my_crc16(buffer, my_len):
    # Функция считают котрольную суммы CRC16-ARC и меняет 1 и 2 байты местами
    polynom = 0xA001
    crc16ret = 0
    for i in range(my_len):
        number = buffer[i]
        crc16ret ^= number
        crc16ret &= 0xFFFF
        for i in range(8):
            if (crc16ret & 0x0001):
                crc16ret = (crc16ret >> 1) ^ polynom
            else:
                crc16ret = crc16ret >> 1
            crc16ret &= 0xFFFF
    crc16ret = hex(crc16ret)
    crc16ret = crc16ret[2:]
    while len(crc16ret) < 4:
        crc16ret = '0' + crc16ret
    first_part = crc16ret[:2]
    second_part = crc16ret[2:]
    crc16ret = first_part + ' ' + second_part + ' '

    return crc16ret


def Fill_table(tableWidget, sheet):
    for i in range(sheet._dimnrows - 1):
        for j in range(sheet._dimncols-1):
            tableWidget.setItem(i, j, QtWidgets.QTableWidgetItem(str(sheet._cell_values[i + 1][j])))


class Books_data():
    def __init__(self, get_text, name='', data=[]):
        self.name = name
        self.data = data
        # Время, 1 бит - 1с
        self.column_1 = []
        # Давление 1 бит - 0.01 бар
        self.column_2 = []
        # Состояние первого клапана
        self.column_3 = []
        # Состояние второго клапана
        self.column_4 = []
        # Состояние третьего клапана
        self.column_5 = []
        # Состояние четвертого клапана
        self.column_6 = []
        # Состояние пятого клапана
        self.column_7 = []
        # Состояние шестого клапана
        self.column_8 = []
        # Состояние ЭД1
        self.column_9 = []
        # Состояние ЭД2
        self.column_10 = []
        self.get_text = get_text

    def read_column_xls(self):
        self.column_1 = []
        self.column_2 = []
        self.column_3 = []
        self.column_4 = []
        self.column_5 = []
        self.column_6 = []
        self.column_7 = []
        self.column_8 = []
        self.column_9 = []
        self.column_10 = []
        try:
            for i in range(self.data._dimnrows - 1):
                for j in range(self.data._dimncols):
                    if j == 0:
                        self.column_1.append(float(self.data._cell_values[i + 1][j]))
                    if j == 1:
                        self.column_2.append(float(self.data._cell_values[i + 1][j]))
                    if j == 2:
                        self.column_3.append(float(self.data._cell_values[i + 1][j]))
                    if j == 3:
                        self.column_4.append(float(self.data._cell_values[i + 1][j]))
                    if j == 4:
                        self.column_5.append(float(self.data._cell_values[i + 1][j]))
                    if j == 5:
                        self.column_6.append(float(self.data._cell_values[i + 1][j]))
                    if j == 6:
                        self.column_7.append(float(self.data._cell_values[i + 1][j]))
                    if j == 7:
                        self.column_8.append(float(self.data._cell_values[i + 1][j]))
                    if j == 8:
                        self.column_9.append(float(self.data._cell_values[i + 1][j]))
                    if j == 9:
                        self.column_10.append(float(self.data._cell_values[i + 1][j]))
        except ValueError:
            self.get_text('Ошибка при чтении данных сценария ' + self.name)
            self.get_text('Проверьте корректность данных')
            self.get_text('Проверьте отсутствие пустых ячеек')
            print (self.data._dimnrows)
            self.get_text('Проверьте кол-во строк = ' + str(self.data._dimnrows))
            self.get_text('Проверьте кол-во колонок = ' + str(self.data._dimncols))
        except IndexError:
            self.get_text('Ошибка при чтении данных сценария ' + self.name)
            self.get_text('Проверьте корректность данных')
            self.get_text('Проверьте отсутствие пустых ячеек')
            self.get_text('Проверьте кол-во строк = ' + str(self.data._dimnrows))
            self.get_text('Проверьте кол-во колонок = ' + str(self.data._dimncols))
        self.len = len(self.column_1)

    def update(self, name, data):
        self.name = name
        self.data = data


class Models_data():
    def __init__(self, get_text, name='', data=[]):
        self.name = name
        self.data = data
        self.column_1 = []
        self.column_2 = []
        self.column_3 = []
        self.column_4 = []

    def read_column_xls(self):
        self.column_1 = []
        self.column_2 = []
        self.column_3 = []
        self.column_4 = []
        # Проверка отправляемых данных на корректность
        for i in range(self.data._dimnrows - 1):
            for j in range(self.data._dimncols):
                try:
                    if j == 0:
                        self.column_1.append(float(self.data._cell_values[i + 1][j]))
                    if j == 1:
                        self.column_2.append(float(self.data._cell_values[i + 1][j]))
                    if j == 2:
                        self.column_3.append(float(self.data._cell_values[i + 1][j]))
                    if j == 3:
                        self.column_4.append(float(self.data._cell_values[i + 1][j]))
                except ValueError:
                    self.get_text('Ошибка при чтении данных графика ' + self.name)
                    self.get_text('Проверьте корректность данных')
        self.len = len(self.column_1)

    def my_clear(self):
        self.column_1 = []
        self.column_2 = []
        self.column_3 = []
        self.column_4 = []

    def set_zero(self):
        self.column_1 = [0]
        self.column_2 = [0]
        self.column_3 = [0]
        self.column_4 = [0]

    def update(self, name, data):
        self.name = name
        self.data = data

    def append_pressure(self, value):
        self.column_1.append(value)

    def append_moment(self, value):
        self.column_2.append(value)

    def append_temperature(self, value):
        self.column_3.append(value)

    def append_time(self, value):
        self.column_4.append(value)


class TryThread(QtCore.QThread):
    def new_run(self):
        print("Abstract class")

    def run(self):
        try:
            self.new_run()
        except:
            print("Ups")


class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=5, dpi=100, plot_number=1):
        # a figure instance to plot on
        self.plot_number = plot_number
        self.fig = plt.figure(figsize=(width, height), dpi=dpi)
        # subplot(x1,x2,x3 или x1x2x3) x1 это количество плотов для графиков по оси X,
        # х2 - по оси Y, x3 это номер графика
        if (plot_number == 1):
            self.title_v = 'Зависимость крутящего момента от давления'
            self.ylabel_plot_1 = 'Крутящий Момент, Н*м'
            self.xlabel_plot_1 = 'Давление, Бар'
        elif (plot_number == 2):
            self.title_v = 'Зависимость крутящего момента и давления от времени'
            self.ylabel_plot_1 = 'Давление, Бар'
            self.ylabel2_plot_1 = 'Крутящий Момент, Н*м'
            self.xlabel_plot_1 = 'Время, с'
        self.axes1 = self.fig.add_subplot(211)
        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        self.plot()
        # позиционирование настроенно для 1 графика
        self.fig.subplots_adjust(left=0.13, right=0.95,
                                 bottom=0.18, top=0.89,
                                 hspace=0.46, wspace=0.2)

    def plot(self, x_1=(), x_2=(), list_plot_1=(), list_plot_2=(), list_plot_3=(), list_plot_4=(), temp_1=0, temp_2=0):
        # отрисовка 1-го графика 100-135 мс, 2-ух 200-250 мс
        try:
            self.fig.clear()
            self.axes1 = self.fig.add_subplot(111)
            # Ось x
            self.x_1 = tuple(x_1)
            self.x_2 = tuple(x_2)
            # Ось у
            self.list_plot_1 = tuple(list_plot_1) # Давление
            self.list_plot_2 = tuple(list_plot_2) # Давление
            self.list_plot_3 = tuple(list_plot_3) # Момент
            self.list_plot_4 = tuple(list_plot_4) # Момент
            self.axes1.grid(True)
            # подписи осей
            self.axes1.set_title(self.title_v)
            self.axes1.set_ylabel(self.ylabel_plot_1)
            self.axes1.set_xlabel(self.xlabel_plot_1)
            if (self.plot_number == 1):
                #self.axes1.axis([0, 4, 0, 350])
                label_1 = 'Исходная модель, ср.Темп=' + str(round(temp_1))
                label_2 = 'Новая модель, ср.Темп=' + str(round(temp_2))
                label_3 = ''
                label_4 = ''
                self.axes1.plot(self.x_1, self.list_plot_1, label=label_1, lw=0.6)
                if self.list_plot_2 != ():
                    self.axes1.plot(self.x_2, self.list_plot_2, label=label_2, lw=0.6)
            elif (self.plot_number == 2):
                label_1 = 'Исходная модель, P1(t)'
                label_2 = 'Новая модель, P2(t)'
                label_3 = 'Исходная модель, M1(t)'
                label_4 = 'Новая модель, M2(t)'
                # создание второй оси у
                self.axes2 = self.axes1.twinx()
                self.axes2.set_ylabel(self.ylabel2_plot_1)
                #axes2.tick_params('y', colors='r')
                self.axes1.plot(self.x_1, self.list_plot_1, label=label_1, lw=0.6)
                if self.list_plot_2 != ():
                    self.axes1.plot(self.x_2, self.list_plot_2, label=label_2, lw=0.6)
                if self.list_plot_3 != ():
                    self.axes2.plot(self.x_1, self.list_plot_3, label=label_3, lw=0.6, color='r')
                if self.list_plot_4 != ():
                    self.axes2.plot(self.x_2, self.list_plot_4, label=label_4, lw=0.6, color='g')
                self.axes2.legend(loc=1)
                self.axes1.legend(loc=2)

            # plt.text(0.16, 0.8, "Средняя температура", size=10,
            #          ha="center", va="center",  transform=self.axes1.transAxes,
            #          bbox=dict(boxstyle="round",
            #                    ec=(1., 0.5, 0.5),
            #                    fc=(1., 0.8, 0.8),
            #                    )
            #          )
            # self.axes1.set_xbound(lower=y[:-10], upper=y[:-1])  # Устанавливаем диапазон оси X
            # self.axes1.set_ybound(lower=2, upper=4.2)  # Устанавливаем диапазон оси Y
            self.axes1.legend(loc=2)
            self.draw()
        except ValueError  :
            # проверь везде ли стоит Self
            print('ValueError при отрисовке данных')
        except :
            print('Ошибка при отрисовке данных')

    def clear(self):
        self.fig.clear()
        self.list_plot_1 = ()
        self.list_plot_2 = ()
        self.axes1 = self.fig.add_subplot(111)
        self.axes1.set_title(self.title_v)
        self.axes1.grid(True)
        self.axes1.set_ylabel(self.ylabel_plot_1)
        self.axes1.set_xlabel(self.xlabel_plot_1)
        self.axes1.plot([])
        self.draw()


class MyPlotThread(TryThread):
    def __init__(self, parent=None, time_delay=1000):
        QtCore.QThread.__init__(self, parent)
        self.time_delay = time_delay
        self.parent = parent
        self.model = False

    def new_model(self, model_state):
        self.model = model_state

    def run(self):
        self.running = True
        while self.running:
            self.msleep(self.time_delay)
            if self.model:
                # statistics.mean не всегда работает
                try:
                    local_temp_1 = statistics.mean(self.parent.model_1.column_3)
                except:
                    local_temp_1 = 0
                self.parent.graf_1.plot(x_1=self.parent.model_1.column_1, x_2=self.parent.data_1.column_1,
                                        list_plot_1=self.parent.model_1.column_2,
                                        list_plot_2=self.parent.data_1.column_2,
                                        temp_1=local_temp_1,
                                        temp_2=0)
            else:
                # statistics.mean не всегда работает
                try:
                    local_temp_1 = statistics.mean(self.parent.model_1.column_3)
                    local_temp_2 = statistics.mean(self.parent.data_1.column_3)
                except:
                    local_temp_1 = 0
                    local_temp_2 = 0
                self.parent.graf_1.plot(x_1=self.parent.model_1.column_1, x_2=self.parent.data_1.column_1,
                                        list_plot_1=self.parent.model_1.column_2,
                                        list_plot_2=self.parent.data_1.column_2,
                                        temp_1=local_temp_1,
                                        temp_2=local_temp_2)
            self.parent.graf_2.plot(x_1=self.parent.model_1.column_4, x_2=self.parent.data_1.column_4,
                                    list_plot_1=self.parent.model_1.column_1, list_plot_2=self.parent.data_1.column_1,
                                    list_plot_3=self.parent.model_1.column_2, list_plot_4=self.parent.data_1.column_2)

    def __del__(self):
        self.wait()

class MyTimerThread(QtCore.QThread):

    def __init__(self, root):
        QtCore.QThread.__init__(self, parent=None)
        self.main = root
        self.time_delay_ms = 100

    def run(self):
        self.running = True
        while self.running:
            if self.main.ErrorComPort:
                print("Остановка")
                self.main.ErrorComPort = False
                self.running = False
            else:
                #i = i + 1
                #print ("Запрос Данных ",i)
                self.main.com_message = [0xBF, 0x04]
                self.main.requestData_14byte = True
                self.main.send_to_com()
                self.msleep(self.time_delay_ms)


    def __del__(self):
        self.wait()


# прием данных по COM порту
class MyComThread(TryThread):
    mysignal_list = QtCore.pyqtSignal(list)
    mysignal_str = QtCore.pyqtSignal(str)

    # mutex = QtCore.QMutex()

    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent

    def get_sent_message(self, sent_message):
        self.sent_message = sent_message

    def run(self):
        try:
            comPorts = serial.tools.list_ports.comports()
            port_device = ''
            for port in comPorts:
                if port.description == self.parent.b_comPorts.currentText():
                    port_device = port.device
            if port_device == '':
                self.parent.get_text('Устройство не найдено')
                return
            self.com_port_name = port_device
            baudrate = 4800
            try:
                self.com = serial.Serial(
                    port=self.com_port_name,
                    baudrate=baudrate)
                # parity=serial.PARITY_NONE,
                # stopbits=serial.STOPBITS_ONE,
                # bytesize=serial.EIGHTBITS)
                my_timeout = 0.07
                self.com.timeout = my_timeout
                if self.com.isOpen:
                    self.parent.get_text('Попытка подключения')
                    # Надо делать кнопку Активной при успешном ответе контроллера
                    self.parent.b_load_book.setEnabled(True)  # Делаем кнопку активной
                    self.parent.b_disconnect.setEnabled(True)  # Делаем кнопку активной

                    self.parent.b_connect.setEnabled(False)  # Делаем кнопку не активной
                    self.parent.data_1.set_zero()
                    self.running = True
                    # Отправка запроса подключения на устройтсво
                    self.parent.com_message = '21 04'
                    self.parent.send_to_com()
                    # Пока поток запущен
                    my_list = []
                    while self.running:
                        # my_data = self.com.read(9)
                        # self.msleep(10)
                        for line in self.com.readline():
                            my_list.append(line)
                        #print (my_list)
                        if (len(my_list)>1):
                            if (my_list[1] == len(my_list)):
                                #print ("Работает")
                                self.mysignal_list.emit(my_list)
                                my_list = []
                            if (len(my_list)>1):
                                if (my_list[1] != 14):
                                    my_list = []
                                    #print ("Обнуление")
                        if (len(my_list)>20):
                            my_list = []
                            #print ("Обнуление")
            except ValueError:
                self.mysignal_str.emit('Ошибка подключения, переподключите устройство')
                self.ErrorComPort = True
        except ValueError:
            self.mysignal_str.emit('Устройство не найдено')
            self.ErrorComPort = True
        except:
            self.mysignal_str.emit('Ошибка подключения, переподключите устройство')
            self.ErrorComPort = True

    def __del__(self):
        self.wait()


# окно добавления сценария
class AddBookWindow(QtWidgets.QDialog, Add_book_Dialog.Ui_Add_Book_Dialog):
    def __init__(self, root, **kwargs):
        super().__init__(root, **kwargs)
        # Передача данных из одного окна в другое работает
        # Выбор родителя
        self.main = root
        self.setupUi(self)
        # генерация списка доступных сценариев
        self.main.generate_combobox(self.comboBox, 'book')
        # путь к файлу
        file_path = os.path.join(self.main.start_file_dir, 'books')
        os.chdir(file_path)
        # открытие Ecxel файла
        rb = xlrd.open_workbook(self.main.book_1.name)
        self.main.book_1.data = rb.sheet_by_index(0)
        # Создание колонок
        self.main.book_1.read_column_xls()
        # заполнение таблицы
        Fill_table(self.tableWidget, self.main.book_1.data)
        # заполнение имени таблицы
        self.lineEdit.setText(self.main.book_1.name[:-4])
        # закрыть окно
        self.b_back.clicked.connect(self.close)
        # удалить сценарий
        self.b_del_book.clicked.connect(self.del_book)
        # сохранить сценарий
        self.b_add_book.clicked.connect(self.save_book)
        # изменение выбранного сценария
        self.comboBox.currentIndexChanged.connect(lambda x: self.focus_book(self.comboBox))
        # изменение данных в ячейке
        self.tableWidget.cellChanged.connect(self.check_input_value)
        self.block_check_input_value = False

    def check_input_value(self, row, column):
        if self.block_check_input_value:
            return
        # проверка на корректность введенных значений
        data = self.tableWidget.item(row, column).text()
        if (data):
            try:
                self.block_check_input_value = True
                data = float(data)
                if column == 0:
                    data = round(data)
                    if data > 1000 or data < 0:
                        QtWidgets.QMessageBox.information(self, 'Ошибка при вводе данных', 'Введенный элемент ' +
                                                          str(data) + ' превышает добустимые пределы: 0-1000 секунд ')
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem('0'))
                        self.block_check_input_value = False
                    else:
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem(str(data)))
                        self.block_check_input_value = False
                if column == 1:
                    data = round(data, 2)
                    if data > 4.00 or data < 0:
                        QtWidgets.QMessageBox.information(self, 'Ошибка при вводе данных', 'Введенный элемент ' +
                                                          str(data) + ' превышает добустимые пределы: 0.00-4.00 Бар, шаг 0.01 Бар ')
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem('0'))
                        self.block_check_input_value = False
                    else:
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem(str(data)))
                        self.block_check_input_value = False
                if column == 2 or column == 3 or column == 4 or column == 5 or column == 6 or column == 7 or column == 8:
                    data = round(data, 1)
                    if data > 1 or data < 0:
                        QtWidgets.QMessageBox.information(self, 'Ошибка при вводе данных', 'Введенный элемент ' +
                                                          str(
                                                              data) + ' превышает добустимые пределы: 0-выкл\закрыт, 1-вкл\открыт ')
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem('0'))
                        self.block_check_input_value = False
                    else:
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem(str(data)))
                        self.block_check_input_value = False
                if column == 9:
                    data = round(data, 1)
                    if data > 1 or data < -1:
                        QtWidgets.QMessageBox.information(self, 'Ошибка при вводе данных', 'Введенный элемент ' +
                                                          str(
                                                              data) + ' превышает добустимые пределы: 0=выкл, 1=вкл , -1=реверс')
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem('0'))
                        self.block_check_input_value = False
                    else:
                        self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem(str(data)))
                        self.block_check_input_value = False
            except ValueError:
                QtWidgets.QMessageBox.information(self, 'Ошибка при вводе данных', 'Введенный элемент ' +
                                                  str(data) + ' не соответствует формату числа: 0.1 или 1')
                self.tableWidget.setItem(row, column, QtWidgets.QTableWidgetItem('0'))
                self.block_check_input_value = False

    def closeEvent(self, evnt):
        # Обновление списка в главном окне
        # установка блокировки изменения book_1
        self.main.block_change_book_1 = True
        # очистка списка
        self.main.b_books.clear()
        first = True
        for find_book in self.main.books_list:
            self.main.b_books.addItem(str(find_book))
            if first:
                self.main.book_1.name = str(find_book)
                first = False
        print('Close AddBookWindow')
        # снятие блокировки изменения book_1
        self.main.block_change_book_1 = False
        super(AddBookWindow, self).closeEvent(evnt)

    def keyPressEvent(self, event):
        print("pressed key " + str(event.key()))

    # def my_close(self):
    #     self._want_to_close = True
    #     self.close()
    #
    # Игнорирование нажатия клавиши Enter
    # def closeEvent(self, evnt):
    #     if self._want_to_close:
    #         super(AddBookWindow, self).closeEvent(evnt)
    #     else:
    #         evnt.ignore()

    def focus_book(self, list_gen):
        if list_gen.currentText() != '':
            self.main.book_1.name = list_gen.currentText()
            # путь к файлу
            file_path = os.path.join(self.main.start_file_dir, 'books')
            os.chdir(file_path)
            # открытие Ecxel файла
            rb = xlrd.open_workbook(self.main.book_1.name)
            self.main.book_1.data = rb.sheet_by_index(0)
            # очистка таблицы
            self.tableWidget.clear()
            self.tableWidget.setHorizontalHeaderLabels(["Время ,с",
                                                        "Давление,\nбар",
                                                        "КЭ1", "КЭ2", "КЭ3", "КЭ4", "КЭ5", "КЭ6",
                                                        "ЭД1", "ЭД2"])
            # заполнение таблицы
            Fill_table(self.tableWidget, self.main.book_1.data)
            # заполнение имени таблицы
            self.lineEdit.setText(self.main.book_1.name[:-4])

    def save_book(self):
        try:
            wb = xlwt.Workbook()
            ws = wb.add_sheet('Book_1')
            ws.write(0, 0, 'Время, с')
            ws.write(0, 1, 'Давление,\nбар')
            ws.write(0, 2, 'КЭ1')
            ws.write(0, 3, 'КЭ2')
            ws.write(0, 4, 'КЭ3')
            ws.write(0, 5, 'КЭ4')
            ws.write(0, 6, 'КЭ5')
            ws.write(0, 7, 'КЭ6')
            ws.write(0, 8, 'ЭД1')
            ws.write(0, 9, 'ЭД2')
            for i in range(self.tableWidget.rowCount() - 1):
                for j in range(10):
                    if self.tableWidget.item(i, j) == None:
                        name = str(self.lineEdit.text())
                        # путь к файлу
                        file_path = os.path.join(self.main.start_file_dir, 'books')
                        os.chdir(file_path)
                        wb.save(name + '.xls')
                        # генерация списка доступных сценариев
                        self.main.generate_combobox(self.comboBox, 'book')
                        # вывод информационного окна
                        QtWidgets.QMessageBox.information(self, 'Сообщение',
                                                          'Сценарий ' + name + '.xls успешно сохранен')
                        return
                    else:
                        val = self.tableWidget.item(i, j).text()
                        ws.write(i + 1, j, val)
        except:
            # вывод информационного окна
            QtWidgets.QMessageBox.information(self, 'Сообщение', 'Ошибка сохранения')

    def del_book(self):
        # вывод информационного окна
        reply = QtWidgets.QMessageBox.question(self, 'Message',
                                               "Вы действительно хотите удалить " + str(
                                                   self.lineEdit.text()) + '.xls ?',
                                               QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                               QtWidgets.QMessageBox.No)

        if reply == QtWidgets.QMessageBox.Yes:
            try:
                name = str(self.lineEdit.text())
                # путь к файлу
                file_path = os.path.join(self.main.start_file_dir, 'books')
                os.chdir(file_path)
                os.remove(name + '.xls')
                # генерация списка доступных сценариев
                self.main.generate_combobox(self.comboBox, 'book')
                # вывод информационного окна
                QtWidgets.QMessageBox.information(self, 'Сообщение',
                                                  'Сценарий ' + name + '.xls успешно удален')
            except:
                # вывод информационного окна
                QtWidgets.QMessageBox.information(self, 'Сообщение', 'Ошибка удаления')


# главное окно
class MyWindow(QtWidgets.QMainWindow, My_1_form.Ui_Stand):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        # изначальный путь к файлу
        self.start_file_dir = os.path.dirname(os.path.abspath(__file__))
        # прикрепление логотипа
        self.setWindowIcon(QtGui.QIcon(self.start_file_dir + os.path.sep + 'logo.png'))
        # Обработка кнопок Меню
        #self.MyMenu.addAction("Справка", self.Help_call)
        self.MyMenu.addAction("Авторы", self.Authors_call)
        # инизиализация данных
        self.pressure_out = 0
        self.temperature = 0
        self.mean_temperature = 0
        self.liquid_level = 0
        self.moment = 0
        self.my_time = 0
        # флаги для этапа эксперимента
        self.exper_run = 0
        self.exper_end = 0
        self.exper_new_data = 0
        self.exper_new_model = 0
        # добавление графика 1
        l1 = QtWidgets.QVBoxLayout(self.w_plot1)
        self.graf_1 = PlotCanvas(self.w_plot1, width=2, height=2, plot_number=1)
        self.toolbar_1 = NavigationToolbar(self.graf_1, self.w_plot1)
        l1.addWidget(self.toolbar_1)
        l1.addWidget(self.graf_1)
        # добавление графика 2
        l2 = QtWidgets.QVBoxLayout(self.w_plot2)
        self.graf_2 = PlotCanvas(self.w_plot2, width=2, height=2, plot_number=2)
        self.toolbar_2 = NavigationToolbar(self.graf_2, self.w_plot2)
        l2.addWidget(self.toolbar_2)
        l2.addWidget(self.graf_2)
        # создаем поток таймер
        self.mySendlerTimerThread = MyTimerThread(self)
        # Ошибка COM порта
        self.ErrorComPort = False
        self.requestData_14byte = False
        #получаем список COM портов
        # ports = serial.tools.list_ports.comports()
        # for port in ports:
        #     self.b_comPorts.addItem(port.description)
        # com_port qSerial
        self.serial = QSerialPort()
        self.serial.setBaudRate(19200)
        self.serial.readyRead.connect(self.readComDataQserial)
        ports = QSerialPortInfo().availablePorts()
        for port in ports: self.b_comPorts.addItem(port.description())
        self.rxTimerCounter = 0
        self.reconnectCounter = 0
        # обновление списка COM портов
        self.b_updateComPorts.clicked.connect(self.updateComPortsList)
        #self.b_comPorts.highlighted.connect(lambda x: (print("qwer")))
        # поток для отрисовки данных
        timer = 200
        self.myPLOTthread_1 = MyPlotThread(self, timer)  # создание потока для цикла отправки
        # создаем поток                    self.myPLOTthread_1.running = False  # Остановка опроса контроллера
        #self.myCOMthread = MyComThread(self)  # создание потока для приема по COM порту
        # Обработчик сигнала от COM emit
        #self.myCOMthread.mysignal_str.connect(self.get_text, QtCore.Qt.QueuedConnection)
        #self.myCOMthread.mysignal_list.connect(self.get_data_from_mc, QtCore.Qt.QueuedConnection)
        # Новые данные
        self.wrote_data = []
        # кнопка стоп красная старт зеленая
        # self.Check_Init_1.setStyleSheet("background-color: rgb(255, 0, 0);\n""color: rgb(255, 255, 255);")
        self.b_stop.setStyleSheet("background-color: rgb(200, 50, 50);")
        self.b_start_1.setStyleSheet("background-color: rgb(50, 200, 50);")
        self.b_load_book.setEnabled(False)  # Делаем кнопку не активной
        self.b_save_graf.setEnabled(False)  # Делаем кнопку не активной
        self.b_start_1.setEnabled(False)  # Делаем кнопку не активной
        self.b_stop.setEnabled(False)  # Делаем кнопку не активной
        self.b_discharge.setEnabled(False)  # Делаем кнопку не активной
        self.b_add_model.setEnabled(False)  # Делаем кнопку не активной
        self.b_disconnect.setEnabled(False)  # Делаем кнопку не активной

        # Создание объекта сценария
        self.book_1 = Books_data(self.get_text)
        # Создание объекта модели
        self.model_1 = Models_data(self.get_text)
        # Создание объекта графика
        self.data_1 = Models_data(self.get_text)
        # Создание объекта полученных данных
        self.mc_recieve_data = Models_data(self.get_text)
        # генерация списка доступных сценариев
        self.generate_combobox(self.b_books, 'book')
        # генерация списка доступных моделей
        self.generate_combobox(self.b_models, 'model')
        # генерация списка доступных графиков
        self.generate_combobox(self.b_graf, 'graf')
        # путь к файлу
        file_path = os.path.join(self.start_file_dir, 'books')
        os.chdir(file_path)
        # открытие Ecxel файла
        rb = xlrd.open_workbook(self.book_1.name)
        self.book_1.data = rb.sheet_by_index(0)
        # Создание колонок
        self.book_1.read_column_xls()
        # путь к файлу
        file_path = os.path.join(self.start_file_dir, 'models')
        os.chdir(file_path)
        # открытие Ecxel файла
        rb_1 = xlrd.open_workbook(self.model_1.name)
        self.model_1.data = rb_1.sheet_by_index(0)
        # Создание колонок
        self.model_1.read_column_xls()
        # путь к файлу
        file_path = os.path.join(self.start_file_dir, 'graf')
        os.chdir(file_path)
        # открытие Ecxel файла
        rb_1 = xlrd.open_workbook(self.data_1.name)
        self.data_1.data = rb_1.sheet_by_index(0)
        # Создание колонок
        self.data_1.read_column_xls()
        # Отрисовка
        self.graf_1.plot(x_1=self.model_1.column_1, x_2=self.data_1.column_1,
                         list_plot_1=self.model_1.column_2, list_plot_2=self.data_1.column_2,
                         temp_1=statistics.mean(self.model_1.column_3),
                         temp_2=statistics.mean(self.data_1.column_3))
        self.graf_2.plot(x_1=self.model_1.column_4, x_2=self.data_1.column_4,
                         list_plot_1=self.model_1.column_1, list_plot_2=self.data_1.column_1,
                         list_plot_3=self.model_1.column_2, list_plot_4=self.data_1.column_2)
        # Выбор сценария, Работает!!!
        # создание нового сценария
        self.b_add_book.clicked.connect(self.add_book)
        # подключиться к устройству
        self.b_connect.clicked.connect(self.connect_com)
        # отключиться от устройства
        self.b_disconnect.clicked.connect(self.disconnect_com)
        # записать параметры и сценарий в контроллер
        self.b_load_book.clicked.connect(self.load_book)
        # открыть график
        # self.b_open_graf.clicked.connect(self.open_graf)
        # старт Испытания 1
        self.b_start_1.clicked.connect(self.experiment_1)
        # стоп
        self.b_stop.clicked.connect(self.stop_all)
        # слив
        self.b_discharge.clicked.connect(self.discharge)
        # сохранение данных
        self.b_save_graf.clicked.connect(self.save_data)
        # изменение выбранного сценария
        self.b_books.currentIndexChanged.connect(lambda x: self.focus_book(self.b_books))
        # изменение выбранной модели
        self.b_models.currentIndexChanged.connect(lambda x: self.focus_model(self.b_models))
        # изменение выбранного графика
        self.b_graf.currentIndexChanged.connect(lambda x: self.focus_graf(self.b_graf))
        # создать новую модель
        self.b_add_model.clicked.connect(self.new_model)
        # включение фильтра сигналов для b_books
        self.b_books.installEventFilter(self)
        # снятие блокировки book_1
        self.block_change_book_1 = False
        # снятие блокировки model_1
        self.block_change_model_1 = False
        # снятие блокировки data_1
        self.block_change_data_1 = False
        # Флаг создания новой модели
        self.new_model_flag = False
        # Флаг приема  корректного сообщения
        self.new_message = False
        # флаг успешного подключения
        self.hello = False
        # Флаг успешной загрузки сценария
        self.book_load = False
        # Флаг успешной загрузки параметров
        self.param_load = False
        # Флаг успешной запуска эксперимента
        self.exper_run = 0
        # Флаг аварийного режима
        self.emergency_mode = False
        # Счетчик для отрисовки
        self.plot_counter = 0
        # тест для оси x
        self.graf_x = []
        # тест для оси y
        self.graf_y = []
        # старое принятое сообщение инициализируем пустой
        self.old_receive_message = []
        # старое значение момента и давления
        self.moment_before = 0
        self.pressure_out_before = 0

    @QtCore.pyqtSlot()
    def readComDataQserial(self):
        #print(self.serial.bytesAvailable())
        self.rxTimerCounter = 0
        if self.requestData_14byte and self.serial.bytesAvailable() == 14:
            rx = self.serial.readAll()
            self.requestData_14byte = False
            #print("Прием", [ord(i) for i in rx])
            self.get_data_from_mc(rx)
        if self.serial.bytesAvailable() >= 4:
            rx = self.serial.readAll()
            #print("Прием", [ord(i) for i in rx])
            self.get_data_from_mc(rx)

    def updateComPortsList(self):
        self.b_comPorts.clear()
        # ports = serial.tools.list_ports.comports()
        # for myPort in ports:
        #     self.b_comPorts.addItem(myPort.description)
        ports = QSerialPortInfo().availablePorts()
        for port in ports: self.b_comPorts.addItem(port.description())

    def check_crc_16(self, my_list):
        # если ошибка связано с индексом массива то возвращаем FALSE
        try:
            len_data = my_list[1]
            control_crc16 = my_crc16(my_list, len_data - 2)
            crc_list = []
            one_byte = ''
            for one_char in control_crc16:
                if one_char == ' ':
                    crc_list.append(int(one_byte, 16))
                    one_byte = ''
                else:
                    one_byte = one_byte + one_char
            if (my_list[len_data - 1] == crc_list[1]) and (my_list[len_data - 2] == crc_list[0]):
                return True
            else:
                return False
        except:
            return False

    def send_book(self):
        # Чтобы конвертировать байты в целое число в Питоне 3 в независимости от числа байт
        # a = int.from_bytes(my_byte_str[0], 'big')
        # b = int.from_bytes(my_byte_str[1], 'big')
        # Проверка отправляемых данных на корректность
        len_list = []
        len_list.append(len(self.book_1.column_1))
        len_list.append(len(self.book_1.column_2))
        len_list.append(len(self.book_1.column_3))
        len_list.append(len(self.book_1.column_4))
        len_list.append(len(self.book_1.column_5))
        len_list.append(len(self.book_1.column_6))
        len_list.append(len(self.book_1.column_7))
        len_list.append(len(self.book_1.column_8))
        len_list.append(len(self.book_1.column_9))
        len_list.append(len(self.book_1.column_10))
        for len_1 in len_list:
            for len_2 in len_list:
                if len_1 != len_2:
                    QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                      'Сценарий ' + self.book_1.name + ' содержит некорректные данные ')
                    return
        index = 0
        # Чтение времени
        for item in self.book_1.column_1:
            try:
                index += 1
                self.block_check_input_value = True
                item = float(item)
                item = round(item)
                if item > 1000 or item < 0:
                    QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                      'Сценарий содержит некорректные данные ' +
                                                      str(item) + 'сторка: ' + str(index) + ' колонка: 1')
                    return
            except ValueError:
                QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                  'Сценарий содержит некорректные данные ' +
                                                  str(item) + 'сторка: ' + str(index) + ' колонка: 1')
                return
        index = 0
        #Чтение давления
        for item in self.book_1.column_2:
            try:
                index += 1
                self.block_check_input_value = True
                item = float(item)
                item = round(item, 2)
                if item > 4.00 or item < 0:
                    QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                      'Сценарий содержит некорректные данные ' +
                                                      str(item) + 'сторка: ' + str(index) + ' колонка: 2')
                    return
            except ValueError:
                QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                  'Сценарий содержит некорректные данные ' +
                                                  str(item) + 'сторка: ' + str(index) + ' колонка: 2')
                return
        #Чтение состояний клапанов
        check_column_list = [self.book_1.column_3, self.book_1.column_4, self.book_1.column_5, self.book_1.column_6,
                             self.book_1.column_7, self.book_1.column_8, self.book_1.column_9]
        column_number = 2
        for column in check_column_list:
            index = 0
            column_number += 1
            for item in column:
                try:
                    index += 1
                    self.block_check_input_value = True
                    item = float(item)
                    item = round(item, 1)
                    if item > 1 or item < 0:
                        QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                      'Сценарий содержит некорректные данные ' +
                                                      str(item) + 'сторка: ' + str(index) + ' колонка: '+ str(column_number))
                        return

                except ValueError:
                    QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                  'Сценарий содержит некорректные данные ' +
                                                  str(item) + 'сторка: ' + str(index) + ' колонка: '+ str(column_number))
                    return
        for item in self.book_1.column_10:
            try:
                index += 1
                self.block_check_input_value = True
                item = float(item)
                item = round(item, 2)
                if item > 1.0 or item < -1:
                    QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                          'Сценарий содержит некорректные данные ' +
                                                          str(item) + 'сторка: ' + str(index) + ' колонка: 10')
                    return
            except ValueError:
                QtWidgets.QMessageBox.information(self, 'Ошибка при чтении данных',
                                                      'Сценарий содержит некорректные данные ' +
                                                      str(item) + 'сторка: ' + str(index) + ' колонка: 10')
                return
        # длина сообщения в два байта
        # 1 - команда
        # 2 - длина сообщения
        # 6 * self.book_1.len - сумма всех байт
        # 2 - контррольная сумма
        int_book_len = int(1 + 2 + (6 * self.book_1.len) + 2)
        my_byte_book_len = int_book_len.to_bytes(2, byteorder='big')
        # команда контроллеру на прием сценария
        self.com_message = [0xCC]
        # Добавляем длину сценария
        self.com_message.append(my_byte_book_len[0])
        self.com_message.append(my_byte_book_len[1])
        # записываем время
        for item in self.book_1.column_1:
            # my_byte_str - это список с двумя байтами
            my_byte_str = (int(item * 10)).to_bytes(2, byteorder='big')
            # Обработка строковых данных
            self.com_message.append(my_byte_str[0])
            self.com_message.append(my_byte_str[1])
        # записываем давление
        for item in self.book_1.column_2:
            # *100 так как 1 бит = 0.01 бар
            calculated_item = (int(item*100.0))
            my_byte_str = calculated_item.to_bytes(2, byteorder='big')
            # Обработка строковых данных
            self.com_message.append(my_byte_str[0])
            self.com_message.append(my_byte_str[1])
        # записываем состояния клапанов
        valve_state_list = [self.book_1.column_3, self.book_1.column_4, self.book_1.column_5,
                            self.book_1.column_6, self.book_1.column_7, self.book_1.column_8]
        for row_number in range(len(self.book_1.column_3)):
            item = 0
            for column_number in range(len(valve_state_list)):
                # если записана единица, то устанавливаем в бите 1
                if valve_state_list[column_number][row_number] == 1:
                    item = item | (1 << column_number)
            my_byte_str = (int(item)).to_bytes(1, byteorder='big')
            # Обработка строковых данных
            self.com_message.append(my_byte_str[0])
        # записываем состояния двигателей
        engine_state_list = [self.book_1.column_9, self.book_1.column_10]
        for row_number in range(len(self.book_1.column_9)):
            item = 0
            for column_number in range(len(engine_state_list)):
                # если записана единица, то устанавливаем в бите 1
                if engine_state_list[column_number][row_number] == 1:
                    item = item | (1 << column_number)
                # если записана минус единица, то устанавливаем в следующем бите 1(работает только для второго двигателя)
                elif engine_state_list[column_number][row_number] == -1:
                    item = item | (1 << (column_number+1))
            my_byte_str = (int(item)).to_bytes(1, byteorder='big')
            # Обработка строковых данных
            self.com_message.append(my_byte_str[0])
        self.send_to_com()

    def get_data_from_mc(self, receive_message_bytes):
        receive_message = [ord(i) for i in receive_message_bytes]
        #print(receive_message)
        #print(receive_message)
        my_list = []
        #print(receive_message)
        new_message_1 = False
        new_message_2 = False
        new_message_3 = False
        try:
            # проверяем что пришли данные нужного размера
            if ((len(receive_message) >= 4) and ((len(receive_message) >= int(receive_message[1])))):
                # если данные пришли полностью то вернет True
                new_message_1 = self.check_crc_16(receive_message)
            # elif ((len(self.old_receive_message) > 4) and ((len(receive_message+self.old_receive_message) == receive_message[1]))):
            #     sum_receive_message = receive_message + self.old_receive_message
            #     # если пришла сначала первая часть потом вторая вернет True
            #     new_message_2 = self.check_crc_16(sum_receive_message)
            #     if not new_message_2:
            #         sum_receive_message = self.old_receive_message + receive_message
            #         # если пришла сначала вторая часть потом первая вернет True
            #         new_message_3 = self.check_crc_16(sum_receive_message)
            # по разному собираем данные в единый массив
            else:
                sum_receive_message_1 = receive_message + self.old_receive_message
                #print('Общее сообщение 1', sum_receive_message_1)
                if len(sum_receive_message_1) >= 4:
                    new_message_2 = self.check_crc_16(sum_receive_message_1)
                sum_receive_message_2 = self.old_receive_message + receive_message
                #print('Общее сообщение 2', sum_receive_message_2)
                # если длина сообщения меньше 4 байт и мы не можем посчитать crc
                if len(sum_receive_message_2) >= 4:
                    new_message_3 = self.check_crc_16(sum_receive_message_2)
            # записываем принятое сообщение как старое
            self.old_receive_message = receive_message
            if new_message_1:
                my_list = receive_message
                #print("Первый вариант ", my_list)
                self.new_message = True
            if new_message_2:
                my_list = sum_receive_message_1
                #print("Второй вариант ",my_list)
                self.new_message = True
            if new_message_3:
                my_list = sum_receive_message_2
                #print("Третий вариант ", my_list)
                self.new_message = True
            #print (my_list)
            if self.new_message:
                # если приняли корректное сообщение сброс старого
                self.old_receive_message = []
                #print(my_list)
                if my_list[0] == 0x21 and my_list[1] == 0x4:
                    self.get_text('Успешное подключение к устройству')
                    self.reconnectCounter = 0
                    self.rxTimerCounter = 0
                    # запуск потока для опроса контроллера
                    if not self.mySendlerTimerThread.isRunning():
                        self.mySendlerTimerThread.start()
                    # активация кнопок
                    self.b_start_1.setEnabled(True)  # Делаем кнопку активной
                    self.b_stop.setEnabled(True)  # Делаем кнопку активной
                    self.b_add_model.setEnabled(True)  # Делаем кнопку активной
                    self.b_discharge.setEnabled(True)  # Делаем кнопку активной
                    self.hello = True

                if my_list[0] == 0xAA and my_list[1] == 0x4:
                    self.get_text('Сценарий: ' + self.book_1.name + ', успешно загружен')
                    # self.get_text('Нажмите Старт для начала испытания')
                    self.book_load = True

                if ((my_list[0] == 0xDD) or (my_list[0] == 0xDF)):
                    # if new_message_3:
                    # print('Данные по 3 варианту')
                    # запись времени
                    if (my_list[0] == 0xDD):
                        self.my_time = ((int(my_list[2]) * 256) + int(my_list[3])) / 10
                        self.lineEdit_12.setText(str(self.my_time))
                    if (my_list[0] == 0xDF):
                        self.my_time = ((int(my_list[2]) * 256) + int(my_list[3])) / 10
                        self.lineEdit_12.setText(str(self.my_time))
                        self.emergency_mode = True
                        self.b_start_1.setEnabled(False)
                        self.b_discharge.setEnabled(False)
                    if (self.my_time == 0):
                        self.emergency_mode = False
                        self.b_start_1.setEnabled(True)
                        self.b_discharge.setEnabled(True)
                    elif (self.my_time > 0):
                        self.b_start_1.setEnabled(False)
                        self.b_discharge.setEnabled(False)
                    # запись Р выходного
                    self.pressure_out = ((int(my_list[4]) * 256) + int(my_list[5])) / 100
                    # Фильтрация скочков давления
                    if abs(self.pressure_out_before - self.pressure_out) > 0.5:
                        self.pressure_out = self.pressure_out_before
                    self.pressure_out_before = self.pressure_out
                    self.lineEdit_7.setText(str(round(self.pressure_out, 2)) + ' Бар')
                    # запись крутящего момента
                    self.moment = ((int(my_list[6]) * 256) + int(my_list[7])) / 2
                    # Фильтрация скачков момента
                    if abs(self.moment_before - self.moment) > 50:
                        self.moment = self.moment_before
                    self.moment_before = self.moment
                    self.lineEdit_10.setText(str(self.moment) + ' Н*м')
                    # запись состояния ключей
                    # Чтение информации по Клапанам
                    # Клапан 1
                    if my_list[8] | 0xFE == 0xFF:
                        self.lineEdit.setText('Открыт')
                        self.lineEdit.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit.setText('Закрыт')
                        self.lineEdit.setStyleSheet("color: rgb(0, 0, 255);")
                    # Клапан 2
                    if my_list[8] | 0xFD == 0xFF:
                        self.lineEdit_2.setText('Открыт')
                        self.lineEdit_2.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_2.setText('Закрыт')
                        self.lineEdit_2.setStyleSheet("color: rgb(0, 0, 255);")
                    # Клапан 3
                    if my_list[8] | 0xFB == 0xFF:
                        self.lineEdit_3.setText('Открыт')
                        self.lineEdit_3.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_3.setText('Закрыт')
                        self.lineEdit_3.setStyleSheet("color: rgb(0, 0, 255);")
                    # Клапан 4
                    if my_list[8] | 0xF7 == 0xFF:
                        self.lineEdit_4.setText('Открыт')
                        self.lineEdit_4.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_4.setText('Закрыт')
                        self.lineEdit_4.setStyleSheet("color: rgb(0, 0, 255);")
                    # Клапан 5
                    if my_list[8] | 0xEF == 0xFF:
                        self.lineEdit_5.setText('Открыт')
                        self.lineEdit_5.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_5.setText('Закрыт')
                        self.lineEdit_5.setStyleSheet("color: rgb(0, 0, 255);")
                    # Клапан 6
                    if my_list[8] | 0xDF == 0xFF:
                        self.lineEdit_6.setText('Открыт')
                        self.lineEdit_6.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_6.setText('Закрыт')
                        self.lineEdit_6.setStyleSheet("color: rgb(0, 0, 255);")
                    if my_list[9] | 0xFE == 0xFF:
                        self.lineEdit_11.setText('Включен')
                        self.lineEdit_11.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_11.setText('Выключен')
                        self.lineEdit_11.setStyleSheet("color: rgb(0, 0, 255);")
                    #print(my_list[9])
                    if my_list[9] | 0xFD == 0xFF:
                        self.lineEdit_13.setText('Включен')
                        self.lineEdit_13.setStyleSheet("color: rgb(0, 255, 0);")
                    elif my_list[9] | 0xFB == 0xFF:
                        self.lineEdit_13.setText('Реверс')
                        self.lineEdit_13.setStyleSheet("color: rgb(0, 255, 0);")
                    else:
                        self.lineEdit_13.setText('Выключен')
                        self.lineEdit_13.setStyleSheet("color: rgb(0, 0, 255);")
                    #Проверки связи с модулем Zetlab
                    if my_list[9] | 0xF7 == 0xFF:
                        self.lineEdit_15.setText('Отсутствует')
                        self.lineEdit_15.setStyleSheet("color: rgb(255, 0, 0);")
                    else:
                        self.lineEdit_15.setText('Установлена')
                        self.lineEdit_15.setStyleSheet("color: rgb(0, 255, 0);")
                    # Проверки связи с РД
                    if my_list[9] | 0xEF == 0xFF:
                        self.lineEdit_16.setText('Отсутствует')
                        self.lineEdit_16.setStyleSheet("color: rgb(255, 0, 0);")
                    else:
                        self.lineEdit_16.setText('Установлена')
                        self.lineEdit_16.setStyleSheet("color: rgb(0, 255, 0);")
                    # запись температуры со сдвигом на 50 градусов
                    self.temperature = my_list[10] - 50
                    self.lineEdit_8.setText(str(self.temperature) + ' °C')
                    # запись средней температуры
                    if len(self.data_1.column_3) > 0:
                        self.mean_temperature = round(statistics.mean(self.data_1.column_3))
                    else:
                        self.mean_temperature = 0
                    #print (my_list)
                    self.lineEdit_14.setText(str(self.mean_temperature) + ' °C')
                    # запись уровня жидкости
                    self.liquid_level = my_list[11]
                    self.lineEdit_9.setText(str(self.liquid_level) + ' %')
                # Испытание завершено
                if my_list[0] == 0xEE and my_list[1] == 0x4:
                    self.get_text('')
                    self.get_text('Испытание завершено за ' + str(round(self.my_time)) + ' секунд')
                    self.exper_end = 1
                    self.exper_run = 0
                if my_list[0] == 0xE1 and my_list[1] == 0x4:
                    self.get_text('')
                    self.get_text(
                        'Испытание завершено по команде оператора на ' + str(round(self.my_time)) + ' секунде')
                    self.exper_end = 1
                    self.exper_run = 0
                if my_list[0] == 0xE2 and my_list[1] == 0x4:
                    self.get_text('')
                    self.get_text('Испытание завершено из-за ошибки "Температура жидкости выше 135°C"')
                    self.get_text('Запущен аварийный режим')
                if my_list[0] == 0xE3 and my_list[1] == 0x4:
                    self.get_text('')
                    self.get_text('Испытание завершено из-за ошибки "Уровень жидкости ниже 10%"')
                    self.get_text('Запущен аварийный режим')
                if my_list[0] == 0xE4 and my_list[1] == 0x4:
                    self.get_text('')
                    self.get_text('Испытание завершено из-за ошибки "Крутящий момент выше 350 Н*м"')
                    self.get_text('Запущен аварийный режим')
                if my_list[0] == 0xE5 and my_list[1] == 0x4:
                    self.get_text('')
                    self.get_text('Испытание завершено из-за ошибки "Давление выше 4.0 Бар"')
                    self.get_text('Запущен аварийный режим')
                if self.my_time > 0 and self.exper_run == 1 and self.exper_new_model == 0:
                    self.exper_new_data = 1
                    self.data_1.append_pressure(self.pressure_out)
                    self.data_1.append_moment(self.moment)
                    self.data_1.append_temperature(self.temperature)
                    self.data_1.append_time(self.my_time)
                    # Поток отрисовки графика
                    if not self.myPLOTthread_1.isRunning():
                        self.myPLOTthread_1.start()
                elif self.my_time > 0 and self.exper_run == 1 and self.exper_new_model == 1:
                    self.model_1.append_pressure(self.pressure_out)
                    self.model_1.append_moment(self.moment)
                    self.model_1.append_temperature(self.temperature)
                    self.model_1.append_time(self.my_time)
                    # Поток отрисовки графика
                    if not self.myPLOTthread_1.isRunning():
                        self.myPLOTthread_1.new_model(True)
                        self.myPLOTthread_1.start()
                self.new_message = False
                #print (my_list)
                my_list = []

                if self.exper_end == 1:
                    if (self.emergency_mode == False):
                        self.b_start_1.setEnabled(True)  # Делаем кнопку активной
                    self.b_load_book.setEnabled(True)  # Делаем кнопку активной
                    self.b_add_model.setEnabled(True)  # Делаем кнопку активной
                    self.b_add_book.setEnabled(True)  # Делаем кнопку активной
                    self.b_disconnect.setEnabled(True)  # Делаем кнопку активной
                    self.b_save_graf.setEnabled(True)  # Делаем кнопку активной
                    self.b_graf.setEnabled(True)  # Делаем кнопку активной
                    self.b_models.setEnabled(True)  # Делаем кнопку активной
                    self.myPLOTthread_1.running = False  # Остановить выполнение потока



        except IndexError:
            print(traceback.format_exc())
            print('Ошибка IndexError' + str(receive_message))
            print('Старое сообщение' + str(self.old_receive_message))
            # записываем принятое сообщение как старое
            self.old_receive_message = receive_message

    def save_data(self):
        try:
            self.wrote_data = [[]]
            if self.new_model_flag:
                for i in range(len(self.model_1.column_1)):
                    self.wrote_data[i].append(self.model_1.column_1[i])
                    self.wrote_data[i].append(self.model_1.column_2[i])
                    self.wrote_data[i].append(self.model_1.column_3[i])
                    self.wrote_data[i].append(self.model_1.column_4[i])
                    self.wrote_data.append([])
                wb = xlwt.Workbook()
                ws = wb.add_sheet('Book_1')
                ws.write(0, 0, 'Давление, Па')
                ws.write(0, 1, 'Момент, Н*м')
                ws.write(0, 2, 'Температура, °C')
                ws.write(0, 3, 'Время, с')
                for i in range(len(self.wrote_data) - 1):
                    for j in range(4):
                        val = self.wrote_data[i][j]
                        ws.write(i + 1, j, val)
                # путь к файлу
                file_path = os.path.join(self.start_file_dir, 'models')
                os.chdir(file_path)
                wb.save(str(self.model_1.name) + '.xls')
                # вывод информационного окна
                QtWidgets.QMessageBox.information(self, 'Сообщение',
                                                  'Модель ' + str(
                                                      self.model_1.name) + '.xls успешно сохранена')
                # генерация списка доступных сценариев
                self.generate_combobox(self.b_models, 'model')
                # снятие блокировки model_1
                self.block_change_model_1 = False
                self.new_model_flag = False
                self.exper_new_model = 0
            elif self.exper_new_data:
                text, ok = QtWidgets.QInputDialog.getText(self, 'Название графика', 'Введите название графика:')
                if ok:
                    self.data_1.name = str(text)
                    for i in range(len(self.data_1.column_1)):
                        self.wrote_data[i].append(self.data_1.column_1[i])
                        self.wrote_data[i].append(self.data_1.column_2[i])
                        self.wrote_data[i].append(self.data_1.column_3[i])
                        self.wrote_data[i].append(self.data_1.column_4[i])
                        self.wrote_data.append([])
                    wb = xlwt.Workbook()
                    ws = wb.add_sheet('Book_1')
                    ws.write(0, 0, 'Давление, Па')
                    ws.write(0, 1, 'Момент, Н*м')
                    ws.write(0, 2, 'Температура, °C')
                    ws.write(0, 3, 'Время, с')
                    for i in range(len(self.wrote_data) - 1):
                        for j in range(4):
                            val = self.wrote_data[i][j]
                            ws.write(i + 1, j, val)
                    # путь к файлу
                    file_path = os.path.join(self.start_file_dir, 'graf')
                    os.chdir(file_path)
                    wb.save(str(self.data_1.name) + '.xls')
                    # вывод информационного окна
                    QtWidgets.QMessageBox.information(self, 'Сообщение',
                                                      'График ' + str(
                                                          self.data_1.name) + '.xls успешно сохранен')
                    self.exper_new_data = 1
                    # снятие блокировки data_1
                    self.block_change_data_1 = False
                    # добавление в список нового графика
                    self.b_graf.addItem(str(self.data_1.name) + '.xls')
            else:
                self.get_text('Нет данных для сохранения')

        except:
            # вывод информационного окна
            QtWidgets.QMessageBox.information(self, 'Сообщение', 'Ошибка сохранения')

    def new_model(self):
        text, ok = QtWidgets.QInputDialog.getText(self, 'Имя модели', 'Введите имя новой модели:')
        if ok:
            self.model_1.my_clear()
            self.model_1.name = str(text)
            self.get_text(' ')
            self.get_text('Модель ' + str(text) + ' создана')
            self.get_text('Запустите испытание для получения данных')
            # Отрисовка

            self.graf_1.plot(x_1=self.model_1.column_1, x_2=self.data_1.column_1,
                             list_plot_1=self.model_1.column_2, list_plot_2=self.data_1.column_2,
                             temp_1=0,
                             temp_2=0)
            self.graf_2.plot(x_1=self.model_1.column_4, x_2=self.data_1.column_4,
                             list_plot_1=self.model_1.column_1, list_plot_2=self.data_1.column_1,
                             list_plot_3=self.model_1.column_2, list_plot_4=self.data_1.column_2)
            self.new_model_flag = True
            # установка блокировки model_1
            self.block_change_model_1 = True
            # установка блокировки data_1
            self.block_change_data_1 = True
            self.b_add_model.setEnabled(False)  # Делаем кнопку не активной
            self.b_graf.setEnabled(False)  # Делаем кнопку не активной
            self.b_models.setEnabled(False)  # Делаем кнопку не активной
            self.exper_new_model = 1

    def open_graf(self):
        # QString getOpenFileName (QWidget parent = None, QString caption = '', QString directory = '', QString filter = '', Options options = 0)
        self.exper_new_data = 0
        # путь к файлу
        file_path = os.path.join(self.start_file_dir, 'graf')
        fname = QtWidgets.QFileDialog.getOpenFileName(self, 'Открыть файл', directory=str(file_path), filter='*.xls')[0]
        file_name = fname[fname.rfind('/') + 1:]
        try:
            rb_2 = xlrd.open_workbook(fname)
            self.data_1.data = rb_2.sheet_by_index(0)
            # Создание колонок
            self.data_1.read_column_xls()
            # Отрисовка
            self.graf_1.plot(x_1=self.model_1.column_1, x_2=self.data_1.column_1,
                             list_plot_1=self.model_1.column_2, list_plot_2=self.data_1.column_2,
                             temp_1=statistics.mean(self.model_1.column_3),
                             temp_2=statistics.mean(self.data_1.column_3))
            self.graf_2.plot(x_1=self.model_1.column_4, x_2=self.data_1.column_4,
                             list_plot_1=self.model_1.column_1, list_plot_2=self.data_1.column_1,
                             list_plot_3=self.model_1.column_2, list_plot_4=self.data_1.column_2)
            self.get_text('Открыт файл ' + file_name)

        except:
            return

    def focus_book(self, list_gen):
        # блокировка при закрытии окна AddWindow
        if not self.block_change_book_1:
            # получение имени
            self.book_1.name = list_gen.currentText()
            # путь к файлу
            file_path = os.path.join(self.start_file_dir, 'books')
            os.chdir(file_path)
            # открытие Ecxel файла
            rb = xlrd.open_workbook(self.book_1.name)
            self.book_1.data = rb.sheet_by_index(0)
            # Создание колонок
            self.book_1.read_column_xls()

    def focus_model(self, list_gen):
        # блокировка при создании новой модели
        if not self.block_change_model_1:
            # получение имени
            self.model_1.name = list_gen.currentText()
            # путь к файлу
            file_path = os.path.join(self.start_file_dir, 'models')
            os.chdir(file_path)
            # открытие Ecxel файла
            rb_1 = xlrd.open_workbook(self.model_1.name)
            self.model_1.data = rb_1.sheet_by_index(0)
            # Создание колонок
            self.model_1.read_column_xls()
            # Отрисовка
            self.data_1.set_zero()
            self.graf_1.plot(x_1=self.model_1.column_1, x_2=self.data_1.column_1,
                             list_plot_1=self.model_1.column_2, list_plot_2=self.data_1.column_2,
                             temp_1=statistics.mean(self.model_1.column_3),
                             temp_2=statistics.mean(self.data_1.column_3))
            self.graf_2.plot(x_1=self.model_1.column_4, x_2=self.data_1.column_4,
                             list_plot_1=self.model_1.column_1, list_plot_2=self.data_1.column_1,
                             list_plot_3=self.model_1.column_2, list_plot_4=self.data_1.column_2)

    def focus_graf(self, list_gen):
        # блокировка при создании новой модели
        if not self.block_change_data_1:
            # получение имени
            self.data_1.name = list_gen.currentText()
            # путь к файлу
            file_path = os.path.join(self.start_file_dir, 'graf')
            os.chdir(file_path)
            # открытие Ecxel файла
            rb_1 = xlrd.open_workbook(self.data_1.name)
            self.data_1.data = rb_1.sheet_by_index(0)
            # Создание колонок
            self.data_1.read_column_xls()
            # Отрисовка
            self.graf_1.plot(x_1=self.model_1.column_1, x_2=self.data_1.column_1,
                             list_plot_1=self.model_1.column_2, list_plot_2=self.data_1.column_2,
                             temp_1=statistics.mean(self.model_1.column_3),
                             temp_2=statistics.mean(self.data_1.column_3))
            self.graf_2.plot(x_1=self.model_1.column_4, x_2=self.data_1.column_4,
                             list_plot_1=self.model_1.column_1, list_plot_2=self.data_1.column_1,
                             list_plot_3=self.model_1.column_2, list_plot_4=self.data_1.column_2)

    def get_text(self, s):
        #self.textBrowser.clear()
        self.textBrowser.append(str(s))

    def add_book(self):
        # в скобках self -> передаем ссылку на родителя, чтобы окно можно было сделать модальным
        self.window_add = AddBookWindow(self)
        # делаем окно модальным
        self.window_add.setWindowModality(QtCore.Qt.WindowModal)
        # self.window_add.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.window_add.show()

    def disconnect_com(self):
        self.mySendlerTimerThread.running = False # Остановка опроса контроллера
        #self.myCOMthread.running = False  # Остановить выполнение потока
        #self.myCOMthread.com.close()
        self.serial.close()

        # Надо делать кнопку Активной при успешном ответе контроллера
        self.b_load_book.setEnabled(False)  # Делаем кнопку не активной

        self.b_connect.setEnabled(True)  # Делаем кнопку активной
        self.b_graf.setEnabled(True)  # Делаем кнопку активной
        self.b_add_book.setEnabled(True)  # Делаем кнопку активной

        self.b_books.setEnabled(True)  # Делаем ComboBox активным
        self.b_models.setEnabled(True)  # Делаем ComboBox активным

        self.b_save_graf.setEnabled(False)  # Делаем кнопку не активной
        self.b_start_1.setEnabled(False)  # Делаем кнопку не активной
        self.b_stop.setEnabled(False)  # Делаем кнопку не активной
        self.b_add_model.setEnabled(False)  # Делаем кнопку не активной
        self.b_disconnect.setEnabled(False)  # Делаем кнопку не активной
        self.b_discharge.setEnabled(False)  # Делаем кнопку не активной

        # sleep(0.2)
        # Сброс параметров
        self.lineEdit.setText('Недоступен')
        self.lineEdit.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_2.setText('Недоступен')
        self.lineEdit_2.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_3.setText('Недоступен')
        self.lineEdit_3.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_4.setText('Недоступен')
        self.lineEdit_4.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_5.setText('Недоступен')
        self.lineEdit_5.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_6.setText('Недоступен')
        self.lineEdit_6.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_7.setText('Недоступен')
        self.lineEdit_8.setText('Недоступен')
        self.lineEdit_9.setText('Недоступен')
        self.lineEdit_10.setText('Недоступен')
        self.lineEdit_11.setText('Недоступен')
        self.lineEdit_11.setStyleSheet("color: rgb(0, 0, 0);")
        self.lineEdit_12.setText('Недоступен')
        self.lineEdit_13.setText('Недоступен')
        self.lineEdit_14.setText('Недоступен')
        self.lineEdit_15.setText('Недоступен')
        self.lineEdit_16.setText('Недоступен')

    def connect_com(self):
        ports = QSerialPortInfo().availablePorts()
        # comPorts = serial.tools.list_ports.comports()
        port_device = ''
        for port in ports:
            if port.description() == self.b_comPorts.currentText():
                # print (port.portName())
                port_device = port.portName()
                self.serial.setPortName(port.portName())
        if port_device == '':
            self.parent.get_text('Устройство не найдено')
            return
        self.com_port_name = port_device
        self.serial.setBaudRate(19200)
        self.serial.open(QIODevice.ReadWrite)
        if self.serial.isOpen():
            self.get_text('Попытка подключения')
            # Надо делать кнопку Активной при успешном ответе контроллера
            self.b_load_book.setEnabled(True)  # Делаем кнопку активной
            self.b_disconnect.setEnabled(True)  # Делаем кнопку активной

            self.b_connect.setEnabled(False)  # Делаем кнопку не активной
            self.data_1.set_zero()
            # Отправка запроса подключения на устройтсво
            self.com_message = '21 04'
            self.send_to_com()
        # if not self.myCOMthread.isRunning():
        #     self.myCOMthread.start()

    def load_book(self):
        self.send_book()

    def generate_combobox(self, gen_list, name):
        # Генерирует список
        gen_list.clear()
        if name == 'book':
            try:
                # путь к файлу
                file_path = os.path.join(self.start_file_dir, 'books')
                os.chdir(file_path)
            except PermissionError:
                print('Ошибка доступа')
            first = True
            self.books_list = []
            for find_book in glob.glob('*.xls'):
                gen_list.addItem(str(find_book))
                self.books_list.append(str(find_book))
                if first:
                    self.book_1.name = str(find_book)
                    first = False
        if name == 'model':
            # путь к файлу
            file_path = os.path.join(self.start_file_dir, 'models')
            os.chdir(file_path)
            first = True
            for find_book in glob.glob('*.xls'):
                gen_list.addItem(str(find_book))
                if first:
                    self.model_1.name = str(find_book)
                    first = False
        if name == 'graf':
            # путь к файлу
            file_path = os.path.join(self.start_file_dir, 'graf')
            os.chdir(file_path)
            first = True
            for find_book in glob.glob('*.xls'):
                gen_list.addItem(str(find_book))
                if first:
                    self.data_1.name = str(find_book)
                    first = False
        # перейти в папку где запущен скрипт
        os.chdir(os.path.abspath(os.path.dirname(os.path.abspath(__file__))))

    def send_to_com(self):
        # 81 00 0D 00 2C AC
        list_message = []
        one_byte = ''
        try:
            message = self.com_message
            # Обработка строковых данных
            if type(message) == str:
                if message[-1] != ' ':
                    message = message + ' '
                for one_char in message:
                    if one_char == ' ':
                        list_message.append(int(one_byte, 16))
                        one_byte = ''
                    else:
                        one_byte = one_byte + one_char
            if type(message) == list:
                list_message = message
            # расчет контрольной суммы для отправляемого сообщения
            crc16 = my_crc16(list_message, len(list_message))
            for one_char in crc16:
                if one_char == ' ':
                    list_message.append(int(one_byte, 16))
                    one_byte = ''
                else:
                    one_byte = one_byte + one_char
            # только для отображения
            # message = message.upper() + crc16.upper()
            # if self.myCOMthread.com.isOpen():
            #     # отправка в COM порт
            #     self.myCOMthread.com.write(list_message)
            # if (self.rxTimerCounter >= 10) and (self.reconnectCounter == 0):
            #     self.reconnectCounter = 1
            #     self.get_text('Связь с устройством потеряна')
            #     self.serial.close()
            # if (self.rxTimerCounter >= 20):
            #     #self.get_text('Попытка переподключения')
            #     self.connect_com()
            #     self.rxTimerCounter = 20
            # self.rxTimerCounter = self.rxTimerCounter + 1
            # print(self.rxTimerCounter)
            #print("Отправка", list_message)
            if self.serial.isOpen():
                #print(list_message)
                # отправка в COM порт
                # print(self.reconnectCounter)
                self.serial.clear()
                self.serial.write(bytearray(list_message))
                self.serial.waitForBytesWritten()

                # для отображения
                # self.sent_message = message[:-1]
                # self.myCOMthread.get_sent_message(self.sent_message)
                #print(list_message)
            # else:
            #     self.get_text('COM порт закрыт')
            #     self.disconnect_com()
        except ValueError:
            self.get_text('Ошибка формата данных')
        except AttributeError:
            self.get_text('объeкт COM port не создан')
            self.ErrorComPort = True
        except:
            self.get_text('Связь с устройством потеряна')
            self.disconnect_com()

    def experiment_1(self):
        self.data_1.my_clear()
        self.b_graf.setEnabled(False)  # Делаем кнопку не активной
        self.b_models.setEnabled(False)  # Делаем кнопку не активной
        self.exper_run = 1
        self.exper_end = 0
        self.com_message = [0xBB, 0x04]
        self.send_to_com()
        self.b_start_1.setEnabled(False)  # Делаем кнопку не активной
        self.b_load_book.setEnabled(False)  # Делаем кнопку не активной
        self.b_add_model.setEnabled(False)  # Делаем кнопку не активной
        self.b_add_book.setEnabled(False)  # Делаем кнопку не активной
        self.b_disconnect.setEnabled(False)  # Делаем кнопку не активной
        self.b_save_graf.setEnabled(False)  # Делаем кнопку не активной
        self.b_discharge.setEnabled(False)  # Делаем кнопку не активной
        self.get_text(' ')
        self.get_text('Запуск эксперимента')

    def stop_all(self):
        self.com_message = [0xEE, 0x04]
        self.send_to_com()
        self.b_start_1.setEnabled(True)  # Делаем кнопку активной
        self.b_load_book.setEnabled(True)  # Делаем кнопку активной
        self.b_add_model.setEnabled(True)  # Делаем кнопку активной
        self.b_add_book.setEnabled(True)  # Делаем кнопку активной
        self.b_disconnect.setEnabled(True)  # Делаем кнопку активной
        self.b_save_graf.setEnabled(True)  # Делаем кнопку активной
        self.b_discharge.setEnabled(True)  # Делаем кнопку  активной

    def discharge(self):
        self.com_message = [0xB1, 0x04]
        self.send_to_com()
        self.b_start_1.setEnabled(False)  # Делаем кнопку активной

    # def Help_call(self):
    #     QtWidgets.QMessageBox.information(self, 'Справка', 'Отсутствует')

    def Authors_call(self):
        QtWidgets.QMessageBox.information(self, 'Авторы', 'Астапенко Дмитрий Александрович\nastap-astap@yandex.ru ')

    #Добавление фона
    # def resizeEvent(self, event):
    #     palette = QtGui.QPalette()
    #     img = QtGui.QImage(self.start_file_dir + os.path.sep + 'Font.jpg')
    #     scaled = img.scaled(self.size(), QtCore.Qt.KeepAspectRatioByExpanding, transformMode=QtCore.Qt.SmoothTransformation)
    #     palette.setBrush(QtGui.QPalette.Window, QtGui.QBrush(scaled))
    #     self.setPalette(palette)

    # Перехват события генерация ComboBox, чтобы избежать зацикливания
    def eventFilter(self, source, event):
        # print (str(event))
        return False
        # print (str(source))
        # if self._want_to_close:
        #     super(AddBookWindow, self).closeEvent(evnt)
        # else:
        #     evnt.ignore()

    def test(self):
        print('OK')
        self.get_text('OK')


app = QtWidgets.QApplication(sys.argv)
window = MyWindow()
window.show()
sys.exit(app.exec_())
