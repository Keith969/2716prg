// *****************************************************************************
// File         [ guiMainWindow.cpp ]
// Description  [ Implementation of the guiMainWindow class ]
// Author       [ Keith Sabine ]
// *****************************************************************************

#include "guiMainWindow.h"

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtSerialPort/QSerialPortInfo>

#include "unistd.h"

// *****************************************************************************
// Function     [ constructor ]
// Description  [ ]
// *****************************************************************************
guiMainWindow::guiMainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    // Set the textEdit font to fixed spacing
    ui.textEdit->setFontFamily("Courier");

    // Set shortcut keys for actions
    ui.actionOpen_HEX_file->setShortcut(tr("Ctrl+O"));
    ui.actionSave_HEX_file->setShortcut(tr("Ctrl+S"));
    ui.actionQuit->setShortcut(tr("Ctrl+Q"));

    // Set connections
    QObject::connect(ui.actionOpen_HEX_file, SIGNAL(triggered()),                this, SLOT(openHexFile()));
    QObject::connect(ui.actionSave_HEX_file, SIGNAL(triggered()),                this, SLOT(saveHexFile()));
    QObject::connect(ui.actionQuit,          SIGNAL(triggered()),                this, SLOT(quit()));
    QObject::connect(ui.initButton,          SIGNAL(pressed()),                  this, SLOT(init()));
    QObject::connect(ui.readButton,          SIGNAL(pressed()),                  this, SLOT(read()));
    QObject::connect(ui.checkButton,         SIGNAL(pressed()),                  this, SLOT(check()));
    QObject::connect(ui.writeButton,         SIGNAL(pressed()),                  this, SLOT(write()));
    QObject::connect(ui.verifyButton,        SIGNAL(pressed()),                  this, SLOT(verify()));
    QObject::connect(ui.loadHexFile,         SIGNAL(clicked()),                  this, SLOT(openHexFile()));
    QObject::connect(ui.saveHexFile,         SIGNAL(clicked()),                  this, SLOT(saveHexFile()));

    // Stuff the serial port combo box
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        QString portName = info.portName();
        // Lets ignore bluetooth stuff, we don't use that
        if (! portName.contains("bluetooth", Qt::CaseInsensitive) && ! portName.contains("BLTH", Qt::CaseInsensitive)) {
            ui.serialPort->addItem(info.portName());
        }
    }

    // Baud rate settings
    ui.baudRate->addItem("115200");
    ui.baudRate->addItem("57600");
    ui.baudRate->addItem("38400");
    ui.baudRate->addItem("19200");
    ui.baudRate->addItem("9600");
    ui.baudRate->addItem("4800");
    ui.baudRate->addItem("2400");

    this->setStatusBar(&m_statusBar);

    QLabel* label = new QLabel("Status");
    statusBar()->insertPermanentWidget(0, label);

    m_ledWidget = new QLedWidget;
    setLedColour(Qt::green);
    setLedPower(true);
    statusBar()->insertPermanentWidget(1, m_ledWidget);

    statusBar()->showMessage("Ready");

    m_HexFile = new hexFile;
    m_HexFile->setMainWindow(this);
    m_initOK = false;

    // Until we have init the baud rate, disable the buttons
    ui.checkButton->setEnabled(false);
    ui.readButton->setEnabled(false);
    ui.writeButton->setEnabled(false);
    ui.verifyButton->setEnabled(false);
}

// *****************************************************************************
// Function     [ cdestructor ]
// Description  [ ]
// *****************************************************************************
guiMainWindow::~guiMainWindow()
{
    delete m_HexFile;
}

// *****************************************************************************
// Function     [ openHexFile ]
// Description  [ Reads a hex file into memory and displays it on the textEdit ]
// *****************************************************************************
void
guiMainWindow::openHexFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open HEX File...", ".", "*.hex");
    m_HexFile->readHex(fileName);

    // Display it in the textEdit widget
    clearText();
    std::vector<hexDataChunk> data = m_HexFile->hexData();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {

        hexDataChunk chunk = *iter;
        QString line;

        // Write the address
        line.append(QString("%1:").arg(chunk.address(), 4, 16, QChar('0')));

        // Write the the data
        for (int32_t i = 0; i < chunk.byteCount(); ++i) {
            QString s = QString(" %1").arg(chunk.data().at(i), 2, 16, QChar('0'));
            line.append(s);
        }
        appendText(line);
    }
}

// *****************************************************************************
// Function     [ saveHexFile ]
// Description  [ Save the contents of the textEdit to a hex file ]
// *****************************************************************************
void
guiMainWindow::saveHexFile()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save HEX File As...", ".", "*.hex");
    if (false == fileName.endsWith(".hex")) {
        fileName += ".hex";
    }

    // Clear any existing hex file
    m_HexFile->clear();

    // Read the textEdit and fill hexfile
    QString text = ui.textEdit->toPlainText();
    QStringList lines = text.split("\n", Qt::SkipEmptyParts);

    uint16_t address=0;
    const int8_t blocksize = 16;
    hexDataChunk chunk;
    bool ok=false;

    // foreach line
    for (auto line_iter = lines.begin(); line_iter != lines.end(); ++line_iter) {
        QString line = *line_iter;
        std::vector<uint8_t> data;
        uint32_t checksum=0;

        // split lines by spaces
        QStringList textlist = line.split(" ", Qt::SkipEmptyParts);
        for (auto token_iter = textlist.begin(); token_iter != textlist.end(); ++token_iter) {

            QString addr = *token_iter;
            // The first item is the address followed by ':'
            if (addr.contains(QChar(':'))) {
                addr.remove(':');
                address = addr.toUShort(&ok, 16);
                chunk.setByteCount(blocksize);
                checksum += blocksize;
                chunk.setAddress(address);
                checksum += address & 0xff;
                checksum += (address >> 8) & 0xff;
                chunk.setRecordType(0);
                checksum += 0;
                token_iter++;
            }

            QString item = *token_iter;
            uint8_t d = (uint8_t) item.toUShort(&ok, 16);
            data.push_back(d);
            checksum += d;
            if (!ok) {
                QString message = QString("Invalid byte %1").arg(item);
                QMessageBox::warning(nullptr, "Not a valid hex value", message);
                return;
            }
        }
        chunk.setData(data);
        uint8_t lsb = ~(checksum & 0xff)+1;
        chunk.setCheckSum(lsb);
        address += blocksize;
        m_HexFile->addChunk(chunk);
    }

    m_HexFile->writeHex(fileName);
}

// *****************************************************************************
// Function     [ getFlowControl ]
// Description  [ Should really always use RTS/CTS ]
// *****************************************************************************
int32_t
guiMainWindow::getFlowControl()
{
    if (ui.flowNone->isChecked())
        return 0;
    /*else if (ui.flowRtsCts->isChecked())*/
        return 1;
}

// *****************************************************************************
// Function     [ init ]
// Description  [ Send a init command to the PIC ]
// *****************************************************************************
void
guiMainWindow::init()
{
    if (m_initOK) {
        QMessageBox::critical(this, "Initialisation", "Init aready done!", QMessageBox::Ok);
        return;
    }
    QString portName = ui.serialPort->currentText();
    int32_t timeout = ui.timeOut->value() * 1000;
    int32_t baudRate = ui.baudRate->currentText().toInt();
    int32_t flowControl = getFlowControl();
    QString devType = ui.deviceType->currentText();

    // Set the serial port
    m_serialPort.close();
    m_serialPort.setPortName(portName);
    m_serialPort.setBaudRate(baudRate);
    m_serialPort.setFlowControl((QSerialPort::FlowControl) flowControl);

    // Now open the port
    if (!m_serialPort.open(QIODevice::ReadWrite)) {
        appendText( QString("Can't open %1, error code %2").arg(portName).arg(m_serialPort.error()) );
        return;
    }

    statusBar()->showMessage(QString("Status: Initialising, connected to port %1.")
                                 .arg(portName));

    setLedColour(Qt::red);

    // Send the cmd, ascii U or 0x55.
    m_serialPort.write(CMD_INIT);

    // Did we get a response?
    if (m_serialPort.waitForBytesWritten(timeout)) {

        // read response from the PIC
        if (m_serialPort.waitForReadyRead(timeout)) {

            QByteArray responseData = m_serialPort.readAll();

            while (m_serialPort.waitForReadyRead(timeout)) {
                responseData += m_serialPort.readAll();
            }
            const QString response = QString::fromUtf8(responseData);

            // The response string should be the baud rate info
            clearText();
            bool ok=true;
            int32_t val=20.0e6 / (4 * (response.toInt(&ok) + 1));
            QString ss = QString("Initialised to %1 baud.").arg(val);
            appendText(ss);
            statusBar()->showMessage("Ready");
            setLedColour(Qt::green);
            m_initOK = true;

            // Enable the buttons
            ui.checkButton->setEnabled(true);
            ui.readButton->setEnabled(true);
            ui.writeButton->setEnabled(true);
            ui.verifyButton->setEnabled(true);

            if (ok == true) {
                statusBar()->showMessage("Initialise OK");
                appendText(QString("Initialise serial link to %1 baud").arg(baudRate));
            }
            else {
                serialError(QString("Failed to initialise serial link to % baud").arg(baudRate));
            }
        } else {
            serialTimeout(QString("Wait read response timeout %1").arg(QTime::currentTime().toString()));
        }
    } else {
        serialTimeout(QString("Wait write request timeout %1").arg(QTime::currentTime().toString()));
    }

    setLedColour(Qt::green);
}

// *****************************************************************************
// Function     [ read ]
// Description  [ Send a read command to the PIC ]
// *****************************************************************************
void
guiMainWindow::read()
{
    if (! m_initOK) {
        QMessageBox::critical(this, "Baud rate", "Init baud rate first!", QMessageBox::Ok);
        return;
    }
    QString portName = ui.serialPort->currentText();
    int32_t timeout = ui.timeOut->value() * 1000;
    int32_t baudRate = ui.baudRate->currentText().toInt();
    int32_t flowControl = getFlowControl();
    QString devType = ui.deviceType->currentText();

    // Set the serial port
    m_serialPort.close();
    m_serialPort.setPortName(portName);
    m_serialPort.setBaudRate(baudRate);
    m_serialPort.setFlowControl((QSerialPort::FlowControl) flowControl);

    // Now open the port
    if (!m_serialPort.open(QIODevice::ReadWrite)) {
        appendText( QString("Can't open %1, error code %2").arg(portName).arg(m_serialPort.error()) );
        return;
    }

    statusBar()->showMessage(QString("Status: Reading, connected to port %1.")
                                 .arg(portName));

    setLedColour(Qt::red);

    // Send the cmd.
    m_serialPort.write(CMD_READ);

    // Did we get a response?
    if (m_serialPort.waitForBytesWritten(timeout)) {

        // read response from the PIC
        if (m_serialPort.waitForReadyRead(timeout)) {

            QByteArray responseData = m_serialPort.readAll();

            while (m_serialPort.waitForReadyRead(timeout)) {
                responseData += m_serialPort.readAll();
            }
            const QString response = QString::fromUtf8(responseData);
            if (response.size() > 2) {
                statusBar()->showMessage("Read OK");
                appendText(QString("Read %1 bytes").arg(response.size()));
            }
            else {
                serialError(QString("Failed to read reponse (%1 bytes read)").arg(response.size()));
            }
        } else {
            serialTimeout(QString("Wait read response timeout %1").arg(QTime::currentTime().toString()));
        }
    } else {
        serialTimeout(QString("Wait cmd write timeout %1").arg(QTime::currentTime().toString()));
    }

    setLedColour(Qt::green);
}

// *****************************************************************************
// Function     [ check ]
// Description  [ Check if the DUT is programmed ]
// *****************************************************************************
void
guiMainWindow::check()
{
    if (! m_initOK) {
        QMessageBox::critical(this, "Baud rate", "Init baud rate first!", QMessageBox::Ok);
        return;
    }
    QString portName = ui.serialPort->currentText();
    int32_t timeout = ui.timeOut->value() * 1000;
    int32_t baudRate = ui.baudRate->currentText().toInt();
    int32_t flowControl = getFlowControl();
    QString devType = ui.deviceType->currentText();

    // Set the serial port
    m_serialPort.close();
    m_serialPort.setPortName(portName);
    m_serialPort.setBaudRate(baudRate);
    m_serialPort.setFlowControl((QSerialPort::FlowControl) flowControl);

    // Now open the port
    if (!m_serialPort.open(QIODevice::ReadWrite)) {
        appendText( QString("Can't open %1, error code %2").arg(portName).arg(m_serialPort.error()) );
        return;
    }

    statusBar()->showMessage(QString("Status: Checking, connected to port %1.")
                                 .arg(portName));

    setLedColour(Qt::red);

    // Send the cmd.
    m_serialPort.write(CMD_READ);

    // Did we get a response?
    if (m_serialPort.waitForBytesWritten(timeout)) {

        // read response from the PIC
        if (m_serialPort.waitForReadyRead(timeout)) {

            QByteArray responseData = m_serialPort.readAll();

            while (m_serialPort.waitForReadyRead(timeout)) {
                responseData += m_serialPort.readAll();
            }
            const QString response = QString::fromUtf8(responseData);
            if (response.size() > 2) {
                statusBar()->showMessage("Check OK");
                appendText(QString("Read %1 bytes").arg(response.size()));
            }
            else {
                serialError(QString("Failed to read reponse (%1 bytes read)").arg(response.size()));
            }
        } else {
            serialTimeout(QString("Wait read response timeout %1").arg(QTime::currentTime().toString()));
        }
    } else {
        serialTimeout(QString("Wait cmd write timeout %1").arg(QTime::currentTime().toString()));
    }

    setLedColour(Qt::green);
}

// *****************************************************************************
// Function     [ write ]
// Description  [ Write the data we read in from a hex file to the PIC ]
// *****************************************************************************
void
guiMainWindow::write()
{
    if (! m_initOK) {
        QMessageBox::critical(this, "Baud rate", "Init baud rate first!", QMessageBox::Ok);
        return;
    }
    if (size() > 0) {
        QString portName = ui.serialPort->currentText();
        int timeout = ui.timeOut->value() * 1000;
        int baudRate = ui.baudRate->currentText().toInt();
        int flowControl = getFlowControl();
        QString devType = ui.deviceType->currentText();

        statusBar()->showMessage(QString("Status: Running, connected to port %1.")
                                     .arg(portName));

        if (devType == "8755") {
            // Send the cmd, followed by the data.
            QString request(CMD_WRTE);

            // Send the data as bytes, using pairs of chars.
            std::vector<hexDataChunk> hData = m_HexFile->hexData();

            for (auto iter = hData.begin(); iter != hData.end(); ++iter) {
                hexDataChunk chunk = *iter;
                std::vector<uint8_t> data = chunk.data();
                uint8_t count = chunk.byteCount();
                for (int8_t i=0; i < count; ++i) {
                    const short d = data.at(i);
                    QString c=QString("%1").arg(d, 2, 16, QChar('0'));
                    request.append(c);
                }
            }

            setLedColour(Qt::red);

            // Send the cmd + data
            const QByteArray requestData = request.toUtf8();
            m_serialPort.write(requestData);

            statusBar()->showMessage("Writing...");
            clearText();
            appendText("Writing data to DUT...");
            qApp->processEvents();

            // Did we get a response?
            if (m_serialPort.waitForBytesWritten(timeout)) {

                // read response from the PIC
                if (m_serialPort.waitForReadyRead(timeout)) {

                    QByteArray responseData = m_serialPort.readAll();

                    while (m_serialPort.waitForReadyRead(timeout)) {
                        responseData += m_serialPort.readAll();
                    }
                    const QString response = QString::fromUtf8(responseData);
                    if (response == "OK") {
                        statusBar()->showMessage("Write OK");
                        appendText(QString("Wrote %1 bytes").arg(requestData.size()));
                    }
                    else {
                        serialError(QString("Failed to write %1 bytes)").arg(requestData.size()));
                    }
                } else {
                    serialTimeout(QString("Wait read response timeout %1").arg(QTime::currentTime().toString()));
                }
            } else {
                serialTimeout(QString("Wait cmd write timeout %1").arg(QTime::currentTime().toString()));
            }

        }
        else if (devType == "2708") {

            // Repeat write 100 times...
            for (int32_t j=0; j < 100; ++j) {
                // Send the cmd, followed by the data.
                QString request(CMD_WRTE);

                // Send the data as bytes, using pairs of chars.
                std::vector<hexDataChunk> hData = m_HexFile->hexData();

                for (auto iter = hData.begin(); iter != hData.end(); ++iter) {
                    hexDataChunk chunk = *iter;
                    std::vector<uint8_t> data = chunk.data();
                    uint8_t count = chunk.byteCount();
                    for (int8_t i=0; i < count; ++i) {
                        const short d = data.at(i);
                        QString c=QString("%1").arg(d, 2, 16, QChar('0'));
                        request.append(c);
                    }
                }

                setLedColour(Qt::red);

                // Send the cmd + data
                const QByteArray requestData = request.toUtf8();
                m_serialPort.write(requestData);

                statusBar()->showMessage(QString("Writing pass %1...").arg(j));
                clearText();
                appendText(QString("Writing pass %1 to DUT").arg(j));
                qApp->processEvents();

                // Did we get a response?
                if (m_serialPort.waitForBytesWritten(timeout)) {

                    // read response from the PIC
                    if (m_serialPort.waitForReadyRead(timeout)) {

                        QByteArray responseData = m_serialPort.readAll();

                        while (m_serialPort.waitForReadyRead(timeout)) {
                            responseData += m_serialPort.readAll();
                        }
                        const QString response = QString::fromUtf8(responseData);
                        if (response == "OK") {
                            statusBar()->showMessage("Write OK");
                            appendText(QString("Wrote %1 bytes").arg(requestData.size()));
                        }
                        else {
                            serialError(QString("Failed to write %1 bytes)").arg(requestData.size()));
                        }
                    } else {
                        serialTimeout(QString("Wait read response timeout %1").arg(QTime::currentTime().toString()));
                    }
                } else {
                    serialTimeout(QString("Wait cmd write timeout %1").arg(QTime::currentTime().toString()));
                }
            }
        }
    }
    else {
        clearText();
        appendText("No HEX data - please open a HEX file!\n");
    }
}

// *****************************************************************************
// Function     [ Verify ]
// Description  [ Verify device ]
// *****************************************************************************
void
guiMainWindow::verify()
{
    if (! m_initOK) {
        QMessageBox::critical(this, "Baud rate", "Init baud rate first!", QMessageBox::Ok);
        return;
    }
    QString portName = ui.serialPort->currentText();
    int32_t timeout = ui.timeOut->value() * 1000;
    int32_t baudRate = ui.baudRate->currentText().toInt();
    int32_t flowControl = getFlowControl();
    QString devType = ui.deviceType->currentText();

    // Set the serial port
    m_serialPort.close();
    m_serialPort.setPortName(portName);
    m_serialPort.setBaudRate(baudRate);
    m_serialPort.setFlowControl((QSerialPort::FlowControl) flowControl);

    // Now open the port
    if (!m_serialPort.open(QIODevice::ReadWrite)) {
        appendText( QString("Can't open %1, error code %2").arg(portName).arg(m_serialPort.error()) );
        return;
    }

    statusBar()->showMessage(QString("Status: Verifying, connected to port %1.")
                                 .arg(portName));

    setLedColour(Qt::red);

    // Send the cmd.
    m_serialPort.write(CMD_READ);

    // Did we get a response?
    if (m_serialPort.waitForBytesWritten(timeout)) {

        // read response from the PIC
        if (m_serialPort.waitForReadyRead(timeout)) {

            QByteArray responseData = m_serialPort.readAll();

            while (m_serialPort.waitForReadyRead(timeout)) {
                responseData += m_serialPort.readAll();
            }
            const QString response = QString::fromUtf8(responseData);
            if (response.size() > 2) {

                clearText();
                // Compare each char of s to the hexfile
                std::vector<hexDataChunk> hexdata = m_HexFile->hexData();
                int32_t j=0;
                bool ok = true;
                for (auto iter = hexdata.begin(); iter != hexdata.end(); ++iter, ++j) {
                    hexDataChunk chunk = *iter;
                    // Write the address of the chunk
                    appendText(QString("$1: ").arg(chunk.address()));
                    j += 6; // skip addr e.g. '0000: '
                    std::vector<uint8_t> data = chunk.data();
                    for (int32_t i=0; i < data.size(); ++i) {
                        // chunk's hex characters given by data.at(i)
                        uint8_t hex_chr = data.at(i);
                        // dev's data
                        QString ss = response.mid(j, 2);
                        // Convert to hex char
                        uint8_t dev_chr = ss.toInt(&ok, 16);
                        // Compare the data. If equal, write the data,
                        // if not equal, write the data in red.
                        if (hex_chr == dev_chr) {
                            appendText(QString("$1 ").arg(ss));
                        }
                        else {
                            ui.textEdit->setTextColor(Qt::red);
                            appendText(QString("$1 ").arg(ss));
                            ui.textEdit->setTextColor(Qt::black);
                            ok = false;
                        }
                    }
                }

                if (ok) {
                    statusBar()->showMessage("Verify OK");
                }
                else {
                    statusBar()->showMessage("Verify failed");
                }
                appendText(QString("Read %1 bytes").arg(response.size()));
            }
            else {
                serialError(QString("Failed to initialise serial link to % baud").arg(baudRate));
            }
        } else {
            serialTimeout(QString("Wait read response timeout %1").arg(QTime::currentTime().toString()));
        }
    } else {
        serialTimeout(QString("Wait cmd write timeout %1").arg(QTime::currentTime().toString()));
    }

    setLedColour(Qt::green);
}

// *****************************************************************************
// Function     [ quit ]
// Description  [ ]
// *****************************************************************************
void
guiMainWindow::quit()
{
    QCoreApplication::exit();
}

// *****************************************************************************
// Function     [ serialError ]
// Description  [ ]
// *****************************************************************************
void
guiMainWindow::serialError(const QString &s)
{
    QString message = QString("Error %1").arg(s);
    QMessageBox::warning(nullptr, "Sender error", message);
    statusBar()->showMessage("Ready");
    setLedColour(Qt::green);
}

// *****************************************************************************
// Function     [ serialTimeout ]
// Description  [ ]
// *****************************************************************************
void
guiMainWindow::serialTimeout(const QString &s)
{
    QString message = QString("Timeout %1").arg(s);
    QMessageBox::warning(nullptr, "Sender timeout", message);
    statusBar()->showMessage("Ready");
    setLedColour(Qt::green);
}

