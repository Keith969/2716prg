// *****************************************************************************
// File         [ hexFile.cpp ]
// Description  [ Implementation of the hexFile class ]
// Author       [ Keith Sabine ]
// *****************************************************************************

#include "hexFile.h"
#include "guiMainWindow.h"

#include <QFile>
#include <QtWidgets/QMessageBox>

// *****************************************************************************
// Function     [ byteCount ]
// Description  [ ]
// *****************************************************************************
 uint8_t
hexDataChunk::byteCount() const
{
    return m_ByteCount;
}

// *****************************************************************************
// Function     [ setByteCount ]
// Description  [ ]
// *****************************************************************************
 void
hexDataChunk::setByteCount(uint8_t n)
{
    m_ByteCount = n;
}

// *****************************************************************************
// Function     [ recordType ]
// Description  [ ]
// *****************************************************************************
 uint8_t
hexDataChunk::recordType() const
{
    return m_RecordType;
}

// *****************************************************************************
// Function     [ setRecordType ]
// Description  [ ]
// *****************************************************************************
 void
hexDataChunk::setRecordType(uint8_t n)
{
    m_RecordType = n;
}

// *****************************************************************************
// Function     [ address ]
// Description  [ ]
// *****************************************************************************
 uint16_t
hexDataChunk::address() const
{
    return m_Address;
}

// *****************************************************************************
// Function     [ setAddress ]
// Description  [ ]
// *****************************************************************************
 void
hexDataChunk::setAddress(uint16_t n)
{
    m_Address = n;
}

// *****************************************************************************
// Function     [ checkSum ]
// Description  [ ]
// *****************************************************************************
 uint8_t
hexDataChunk::checkSum() const
{
    return m_Checksum;
}

// *****************************************************************************
// Function     [ setCheckSum ]
// Description  [ ]
// *****************************************************************************
 void
hexDataChunk::setCheckSum(uint8_t n)
{
    m_Checksum = n;
}

// *****************************************************************************
// Function     [ data ]
// Description  [ ]
// *****************************************************************************
 std::vector<uint8_t> &
hexDataChunk::data()
{
    return m_Data;
}

// *****************************************************************************
// Function     [ setData ]
// Description  [ ]
// *****************************************************************************
 void
hexDataChunk::setData(const std::vector<uint8_t> &d)
{
    m_Data = d;
}

// *****************************************************************************
// Function     [ setMainWindow ]
// Description  [ ]
// *****************************************************************************
void
hexFile::setMainWindow(guiMainWindow* win)
{
    m_MainWindow = win;
}

// *****************************************************************************
// Function     [ mainWindow ]
// Description  [ ]
// *****************************************************************************
guiMainWindow*
hexFile::mainWindow()
{
    return m_MainWindow;
}

// *****************************************************************************
// Function     [ size ]
// Description  [ ]
// *****************************************************************************
size_t
hexFile::size()
{
    return m_HexData.size();
}

// *****************************************************************************
// Function     [ hexData ]
// Description  [ ]
// *****************************************************************************
std::vector<hexDataChunk> &
hexFile::hexData()
{
    return m_HexData;
}

// *****************************************************************************
// Function     [ addChunk ]
// Description  [ ]
// *****************************************************************************
void
hexFile::addChunk(const hexDataChunk &c)
{
    m_HexData.push_back(c);
}

// *****************************************************************************
// Function     [ clear ]
// Description  [ ]
// *****************************************************************************
void
hexFile::clear()
{
    m_HexData.clear();
}

// *****************************************************************************
// Function     [ readHex ]
// Description  [ Read a hex file, setting up the data ]
// *****************************************************************************
bool
hexFile::readHex(const QString& hexFileName)
{
    mainWindow()->clearText();
    QFile fi(hexFileName);
    if (fi.open(QIODevice::ReadOnly)) {

        // Initialise some stuff
        uint32_t lineNum = 0;
        bool ok = true;
        QString s;

        while (!fi.atEnd()) {
            hexDataChunk chunk;

            // Read a line
            QString line = fi.readLine();
            // Append it to the msg area
            mainWindow()->appendText(line.remove("\n"));

            // Get the start character
            int32_t index = line.indexOf(QChar(':'));
            if (index < 0) {
                QString message = QString("Not a valid HEX file at line %1").arg(lineNum);
                QMessageBox::warning(nullptr, "Not a HEX file", message);
                return false;
            }

            // The sum of all bytes following ':'
            uint32_t checkSum = 0;

            // Get byte count
            s = line.sliced(++index, 2);
            int8_t byteCount = s.toInt(&ok, 16);
            if (ok) {
                chunk.setByteCount(byteCount);
                checkSum += byteCount;
            }
            else {
                QString message = QString("Invalid byte count %s at line %1").arg(s).arg(lineNum);
                QMessageBox::warning(nullptr, "Invalid byte count", message);
                return false;
            }
            index += 2;

            // Get the address
            s = line.sliced(index, 2);
            uint8_t hi = s.toInt(&ok, 16);
            if (ok) {
                checkSum += hi;
                index += 2;
            }
            else {
                QString message = QString("Invalid hi address %1 at line %2").arg(s).arg(lineNum);
                QMessageBox::warning(nullptr, "Invalid address", message);
                return false;
            }
            s = line.sliced(index, 2);
            uint8_t lo = s.toInt(&ok, 16);
            if (ok) {
                checkSum += lo;
                index += 2;
                int16_t address = (hi << 8) + lo;
                chunk.setAddress(address);
            }
            else {
                QString message = QString("Invalid lo address %1 at line %2").arg(s).arg(lineNum);
                QMessageBox::warning(nullptr, "Invalid address", message);
                return false;
            }


            // Get the record type
            s = line.sliced(index, 2);
            int8_t recType = s.toInt(&ok, 16);
            if (ok) {
                if (recType == 1) {
                    // this is an end of file
                    fi.close();
                    return true;
                }
                else if (recType == 0) {
                    chunk.setRecordType(recType);
                    checkSum += recType;
                }
                else {
                    QString message = QString("Invalid record type %1 at line %2").arg(s).arg(lineNum);
                    QMessageBox::warning(nullptr, "Invalid record type", message);
                    return false;
                }
            }
            else {
                QString message = QString("Invalid record type %1 at line %2").arg(s).arg(lineNum);
                QMessageBox::warning(nullptr, "Invalid record type", message);
                return false;
            }
            index += 2;

            // Get the bytes
            std::vector<uint8_t> data;
            for (int8_t i = 0; i < byteCount; ++i) {
                s = line.sliced(index, 2);
                int16_t val = s.toInt(&ok, 16);
                if (ok) {
                    data.push_back(val);
                    checkSum += val;
                }
                else {
                    QString message = QString("Invalid byte %s at line %1").arg(s).arg(lineNum);
                    QMessageBox::warning(nullptr, "Invalid byte", message);
                    return false;
                }
                index += 2;
            }
            chunk.setData(data);

            // Compute the checksum, which is the two's complement of the lsb of the sum of all bytes.
            uint8_t lsb = ~(checkSum & 0xff)+1;

            // Get the checksum and compare, just in case
            s = line.sliced(index, 2);
            uint8_t cs = s.toInt(&ok, 16);
            if (ok) {
                if (cs != lsb) {
                    QString message = QString("Invalid checksum at line %1").arg(lineNum);
                    QMessageBox::warning(nullptr, "Invalid checksum", message);
                    return false;
                }
                else {
                    chunk.setCheckSum(cs);
                }
            }

            // Add to the hex buffer
            m_HexData.push_back(chunk);

            // bump line number
            lineNum++;

        } // while not at end of file

    }
    fi.close();
    return true;
}

// *****************************************************************************
// Function     [ writeHex ]
// Description  [ Write the hexData buffer to disk ]
// *****************************************************************************
bool
hexFile::writeHex(const QString& hexFileName)
{
    QFile fi(hexFileName);
    if (fi.open(QIODevice::WriteOnly)) {
        for (auto iter = m_HexData.begin(); iter != m_HexData.end(); ++iter) {
            hexDataChunk chunk = *iter;
            QString line, s;
            // Start with identifier
            line.append(QChar(':'));
            // then the byte count
            s = QString("%1").arg(chunk.byteCount(), 2, 16, QChar('0'));
            line.append(s.toUpper());
            // then the address
            s = QString("%1").arg(chunk.address(), 4, 16, QChar('0'));
            line.append(s.toUpper());
            // then the record type
            s = QString("%1").arg(chunk.recordType(), 2, 16, QChar('0'));
            line.append(s.toUpper());
            // then the data
            for (int32_t i = 0; i < chunk.byteCount(); ++i) {
                s = QString("%1").arg(chunk.data().at(i), 2, 16, QChar('0'));
                line.append(s.toUpper());
            }
            // lastly the checksum and a newline
            s = QString("%1\n").arg(chunk.checkSum(), 2, 16, QChar('0'));
            line.append(s.toUpper());
            fi.write(line.toLatin1());
        }
        // At the end, write a zero padded line with recType 1
        QString line(":00000001FF\n");
        fi.write(line.toLatin1());
    }
    fi.close();
    return true;
}
