// Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef SENDERTHREAD_H
#define SENDERTHREAD_H

// *****************************************************************************
// File         [ SenderThread.h ]
// Description  [ Implementation of the SenderThread class ]
// Author       [ Keith Sabine ]
// *****************************************************************************

#include <QMutex>
#include <QThread>
#include <QWaitCondition>

// *****************************************************************************
// Class        [ SenderThread ]
// Description  [ ]
// *****************************************************************************
class SenderThread : public QThread
{
    Q_OBJECT

public:
    explicit SenderThread(QObject *parent = nullptr);
    ~SenderThread();

    void                      transaction(const QString &portName,
                                            const QString &request,
                                            int waitTimeout=10000,
                                            int baudRate=115200,
                                            int flowControl=0,
                                            bool program=false);
    size_t                    bytesSent() {return m_bytesSent;}
    size_t                    bytesReceived() {m_bytesReceived;}

signals:
    void                      response(const QString &s);
    void                      error(const QString &s);
    void                      timeout(const QString &s);

private:
    void                      run() override;

    QString                   m_portName;
    QString                   m_request;
    int                       m_waitTimeout = 0;
    QMutex                    m_mutex;
    QWaitCondition            m_cond;
    bool                      m_quit = false;
    int32_t                   m_baudrate = 115200;
    int32_t                   m_flowControl = 0;
    size_t                    m_bytesSent;
    size_t                    m_bytesReceived;
    bool                      m_program;
};

#endif /* SENDERTHREAD_H */
