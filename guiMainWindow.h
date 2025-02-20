#ifndef GUIMAINWINDOW_H
#define GUIMAINWINDOW_H

// *****************************************************************************
// File         [ guiMainWindow.h ]
// Description  [ Implementation of the guiMainWindow class ]
// Author       [ Keith Sabine ]
// *****************************************************************************

#include <QtWidgets/QMainWindow>
#include "ui_guiMainWindow.h"
#include "hexFile.h"
#include "senderthread.h"
#include "qLedWidget.h"

#define CMD_DONE "$0"
#define CMD_READ "$1"
#define CMD_WRTE "$2"
#define CMD_CHEK "$3"
#define CMD_INIT "U"

typedef enum {
    op_none,
    op_read,
    op_write,
    op_verify,
    op_check,
    op_init
} mode;

// *****************************************************************************
// Class        [ guiMainWindow ]
// Description  [ ]
// *****************************************************************************
class guiMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    guiMainWindow(QWidget *parent = Q_NULLPTR);
    ~guiMainWindow();

public slots:
    void                   openHexFile();
    void                   saveHexFile();
    void                   init();
    void                   quit();
    void                   read();
    void                   check();
    void                   write();
    void                   verify();

    void                   senderShowResponse(const QString &);
    void                   senderProcessError(const QString &);
    void                   senderProcessTimeout(const QString &);

    void                   appendText(const QString& s) { ui.textEdit->append(s); }
    void                   clearText() { ui.textEdit->clear(); }

    void                   setLedPower(bool pwr) { m_ledWidget->setPower(pwr); }
    void                   setLedColour(const QColor& color) { m_ledWidget->setColour(color); }

private:
    size_t                 size() {return m_HexFile->size();}
    int32_t                getFlowControl();

    // ui
    Ui::guiMainWindowClass ui;

    // The hex file structure
    hexFile              * m_HexFile;

    // Sender thread
    SenderThread           m_senderThread;

    // Status bar
    QStatusBar             m_statusBar;
    QLabel                 m_statusMsg;
    QLedWidget           * m_ledWidget;

    // Operation mode
    mode                   m_mode;
    bool                   m_initOK;
};

#endif /* GUIMAINWINDOW_H */
