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

    // Sender read thread
    QObject::connect(&m_senderThread,        SIGNAL(response(const QString &)),  this, SLOT(senderShowResponse(const QString &)));
    QObject::connect(&m_senderThread,        SIGNAL(error(const QString &)),     this, SLOT(senderProcessError(const QString &)));
    QObject::connect(&m_senderThread,        SIGNAL(timeout(const QString &)),   this, SLOT(senderProcessTimeout(const QString &)));

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
    m_mode = op_none;
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
        // if done
        if (line.startsWith("DUT") || line.isEmpty()) break;
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
    QString portName = ui.serialPort->currentText();
    int timeout = ui.timeOut->value() * 1000;
    int baudRate = ui.baudRate->currentText().toInt();
    int flowControl = getFlowControl();
    m_mode = op_init;

    statusBar()->showMessage(QString("Status: Initialising, connected to port %1.")
                                 .arg(portName));

    setLedColour(Qt::red);
    QString devType = ui.deviceType->currentText();

    // Send the cmd.
    m_senderThread.transaction(portName, CMD_INIT, devType, timeout, baudRate, flowControl);

    statusBar()->showMessage("Initialising...");
    clearText();
    appendText("Initialising programmer...");
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
    int timeout = ui.timeOut->value() * 1000;
    int baudRate = ui.baudRate->currentText().toInt();
    int flowControl = getFlowControl();
    m_mode = op_read;

    statusBar()->showMessage(QString("Status: Running, connected to port %1.")
                                 .arg(portName));

    setLedColour(Qt::red);
    QString devType = ui.deviceType->currentText();

    // Send the cmd.
    m_senderThread.transaction(portName, CMD_READ, devType, timeout, baudRate, flowControl);

    statusBar()->showMessage("Reading...");
    clearText();
    appendText("Reading data from DUT...");
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
    int timeout = ui.timeOut->value() * 1000;
    int baudRate = ui.baudRate->currentText().toInt();
    int flowControl = getFlowControl();
    m_mode = op_check;

    statusBar()->showMessage(QString("Status: Running, connected to port %1.")
                                 .arg(portName));
    setLedColour(Qt::red);
    QString devType = ui.deviceType->currentText();

    // Send the cmd.
    m_senderThread.transaction(portName, CMD_CHEK, devType, timeout, baudRate, flowControl);

    statusBar()->showMessage("Checking...");
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
        m_mode = op_write;

        statusBar()->showMessage(QString("Status: Running, connected to port %1.")
                                     .arg(portName));

        setLedColour(Qt::red);
        clearText();
        appendText("Writing data to DUT...");

        // Send the data as bytes, using pairs of chars.
        std::vector<hexDataChunk> hData = m_HexFile->hexData();

        QString devType = ui.deviceType->currentText();

        // Send the write cmd, followed by the data.
        QString request(CMD_WRTE);

        statusBar()->showMessage("Status: Running, write.");

        // Set up the data then start the thread
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
        m_senderThread.transaction(portName, request, devType, timeout, baudRate, flowControl, true);
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
    if (size() > 0) {
        QString portName = ui.serialPort->currentText();
        int timeout = ui.timeOut->value() * 1000;
        int baudRate = ui.baudRate->currentText().toInt();
        int flowControl = getFlowControl();
        m_mode = op_verify;

        statusBar()->showMessage(QString("Status: Running, connected to port %1.")
                                     .arg(portName));

        setLedColour(Qt::red);
        QString devType = ui.deviceType->currentText();

        // Send the cmd.
        m_senderThread.transaction(portName, CMD_READ, devType, timeout, baudRate, flowControl);

        statusBar()->showMessage("Verifying...");
        clearText();
        appendText("Verifying DUT data...");
    }
    else {
        clearText();
        appendText("No HEX data - please open a HEX file!\n");
    }
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
// Function     [ senderShowResponse ]
// Description  [ Show the thread response string in the text area. ]
// *****************************************************************************
void
guiMainWindow::senderShowResponse(const QString &s)
{
    // If we are doing a read, clear the textEdit then display the response.
    if (m_mode == op_read || m_mode == op_write || m_mode == op_check) {
        clearText();
        appendText(s);
        statusBar()->showMessage("Ready");
        setLedColour(Qt::green);
    }
    // If a verify, compare with m_HexFile
    else if (m_mode == op_verify) {
        clearText();
        // Compare each char of s to the hexfile
        std::vector<hexDataChunk> hexdata = m_HexFile->hexData();
        int32_t j=0;
        int32_t bad=0;
        bool ok = true;
        for (auto iter = hexdata.begin(); iter != hexdata.end(); ++iter, ++j) {
            hexDataChunk chunk = *iter;
            // Write the address of the chunk
            QString ss; ss.setNum(chunk.address(), 16);
            ui.textEdit->insertPlainText(QString("%1: ").arg(ss,4,QChar('0')));
            j += 6; // skip addr e.g. '0000: '
            std::vector<uint8_t> data = chunk.data();
            for (int32_t i=0; i < data.size(); ++i) {
                // chunk's hex characters given by data.at(i)
                uint8_t hex_chr = data.at(i);
                // dev's data is 2 chars
                QString ss = s.mid(j, 2);
                // Convert to hex char
                uint8_t dev_chr = ss.toInt(&ok, 16);
                // Compare the data. If equal, write the data,
                // if not equal, write the data in red.
                if (hex_chr == dev_chr) {
                    ui.textEdit->insertPlainText(QString("%1 ").arg(ss,2,QChar('0')));
                }
                else {
                    ui.textEdit->setTextColor(Qt::red);
                    ui.textEdit->insertPlainText(QString("%1 ").arg(ss,2,QChar('0')));
                    ui.textEdit->setTextColor(Qt::black);
                    bad++;
                }
                // increment j pointer
                if (i == 15) {
                    j += 2;
                } else {
                    j += 3;
                }
            }
            ui.textEdit->insertPlainText("\n");
        }
        if (bad != 0) {
            appendText(QString("DUT has %1 differences with hex file!").arg(bad));
        }
        else {
            appendText("DUT verified correct.");
        }
    }
    else if (m_mode == op_init) {
        // The response string should be the baud rate info
        clearText();
        bool ok=true;
        int32_t val=20.0e6 / (4 * (s.toInt(&ok) + 1));
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
    }
}

// *****************************************************************************
// Function     [ senderProcessError ]
// Description  [ ]
// *****************************************************************************
void
guiMainWindow::senderProcessError(const QString &s)
{
    QString message = QString("Error %1").arg(s);
    QMessageBox::warning(nullptr, "Sender error", message);
    statusBar()->showMessage("Ready");
    setLedColour(Qt::green);
}

// *****************************************************************************
// Function     [ senderProcessTimeout ]
// Description  [ ]
// *****************************************************************************
void
guiMainWindow::senderProcessTimeout(const QString &s)
{
    QString message = QString("Timeout %1").arg(s);
    QMessageBox::warning(nullptr, "Sender timeout", message);
    statusBar()->showMessage("Ready");
    setLedColour(Qt::green);
}

