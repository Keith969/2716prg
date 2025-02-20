#ifndef HEXFILE_H
#define HEXFILE_H

// *****************************************************************************
// File         [ hexFile.h ]
// Description  [ Implementation of the hexFile class ]
// Author       [ Keith Sabine ]
// *****************************************************************************

#include <QString>

class guiMainWindow;

// *****************************************************************************
// Class        [ hexDataChunk ]
// Description  [ A chunk of hex data ]
// *****************************************************************************
class hexDataChunk
{
public:
    hexDataChunk():
        m_ByteCount(0),
        m_RecordType(0),
        m_Address(0),
        m_Checksum(0) {}
    ~hexDataChunk() {}

    uint8_t                   byteCount() const;
    void                      setByteCount(uint8_t n);
    uint8_t                   recordType() const;
    void                      setRecordType(uint8_t n);
    uint16_t                  address() const;
    void                      setAddress(uint16_t n);
    uint8_t                   checkSum()const;
    void                      setCheckSum(uint8_t n);
    std::vector<uint8_t>    & data();
    void                      setData(const std::vector<uint8_t> &d);

private:
    uint8_t                   m_ByteCount;
    uint8_t                   m_RecordType;
    uint16_t                  m_Address;
    uint8_t                   m_Checksum;
    std::vector<uint8_t>      m_Data;
};


// *****************************************************************************
// Class        [ hexFile ]
// Description  [ A file of hexDataChunks ]
// *****************************************************************************
class hexFile
{
public:
    hexFile() {}
    ~hexFile() {}

    bool                      readHex(const QString& hexFileName);
    bool                      writeHex(const QString& hexFileName);
    void                      setMainWindow(guiMainWindow* win);
    guiMainWindow           * mainWindow();
    size_t                    size();
    std::vector<hexDataChunk> &hexData();
    void                      addChunk(const hexDataChunk &c);
    void                      clear();

private:
    std::vector<hexDataChunk> m_HexData;
    guiMainWindow           * m_MainWindow;
};

#endif /* HEXFILE_H */
