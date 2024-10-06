#include "Gripper.h"
#include <QHostAddress>
#include <QDebug>
#include <QTimer>
#include <QEventLoop>

Gripper::Gripper(QObject *parent) :
    QObject(parent),
    socket(new QTcpSocket(this)) // Initialize the QTcpSocket
{
    // Connect signals to the slots
    connect(socket, &QTcpSocket::connected, this, &Gripper::onConnected);
    connect(socket, &QTcpSocket::disconnected, this, &Gripper::onDisconnected);
    connect(socket, &QTcpSocket::readyRead, this, &Gripper::onReadyRead);
    connect(socket, qOverload<QAbstractSocket::SocketError>(&QTcpSocket::errorOccurred), this, &Gripper::onError);
}

Gripper::~Gripper()
{
    // Clean up
    socket->disconnectFromHost();
    delete socket;
}

// ---- TCP Related ----
void Gripper::connectToServer(const QString &host, quint16 port)
{
    // Connect to the server
    qDebug() << "Connecting to server: " << host << ":" << port;
    socket->connectToHost(host, port);

    // Wait for the connection to be established or fail
    if (!socket->waitForConnected(5000)) { // wait for 5 seconds
        qDebug() << "Failed to connect to server: " << socket->errorString();
    }
}

void Gripper::disconnectFromServer()
{
    // Disconnect from the server
    if (socket->isOpen()) {
        qDebug() << "Disconnecting from server";
        socket->disconnectFromHost();
    }
}

void Gripper::sendData(const QByteArray &data)
{
    // Check if the socket is in the connected state
    if (socket->state() == QAbstractSocket::ConnectedState) {
        // Send data with a newline character if necessary
        socket->write(data + "\n");

        // Wait for data to be written
        if (!socket->waitForBytesWritten(3000)) {
            qDebug() << "Failed to send data: " << socket->errorString();
        }
        else {
            qDebug() << "Data sent successfully: " << data;
        }
    }
    else{
        qDebug() << "Socket is not connected!";
    }
}

void Gripper::onConnected()
{
    // Handle when the connection is established
    qDebug() << "Connected to server!";
    statusConnection = true; // Update connection status
}

bool Gripper::isConnected() const
{
    return statusConnection; // Return connection status
}

void Gripper::onDisconnected()
{
    // Handle when the connection is closed
    qDebug() << "Disconnected from server.";
    statusConnection = false; // Update connection status
}

void Gripper::onReadyRead()
{
    // Handle incoming data
    mReadDataArray = socket->readAll(); // Read all available data
    // Convert QByteArray to QString
    mReadDataString = QString::fromUtf8(mReadDataArray);
    mReadDataString.remove(mReadDataString.length() - 1, 2);
    //qDebug() << "Converted QString: " << mReadDataString;
}

void Gripper::onError(QAbstractSocket::SocketError socketError)
{
    // Handle socket errors
    qDebug() << "Socket error: " << socketError << " - " << socket->errorString();
}



// ---- Gripper Related ----
bool Gripper::Command(QString command) {
    // Step 1: Send the command
    mSendData = command; // Use the command provided as a parameter
    sendData(mSendData.toUtf8()); // Send the command as a QByteArray

    // Step 2: Prepare to read the response
    // Trim the command into its basic, so it can be read in the response from the gripper
    int lastIndex = command.lastIndexOf('(');  // Find the last '('
    QString trimmedCommand;
    if (lastIndex != -1) {
        trimmedCommand = command.remove(lastIndex, command.length() - lastIndex);  // Remove from '(' to end
    }
    //If lastIndex is -1, then no '(' was found and the trimmed command is the same as the command
    else {
        trimmedCommand = command;
    }
    //Construct the expected responses
    QString expectedAck = "ACK " + trimmedCommand;
    QString expectedFin = "FIN " + trimmedCommand;
    QEventLoop loop; // Create an event loop to wait for signals
    QTimer timer; // Create a timer for timeout
    bool ackReceived = false;
    bool finReceived = false;

    // Step 3: Connect signals to handle incoming data
    connect(socket, &QTcpSocket::readyRead, this, [&]() {
        qDebug() << "Received response: " << mReadDataString;

        // Check for expected ACK response
        if (mReadDataString.trimmed() == expectedAck) {
            qDebug() << "Command " << trimmedCommand << " Acknowledged";
            ackReceived = true;
        }

        // Check for expected FIN response
        if (mReadDataString.trimmed() == expectedFin) {
            qDebug() << "Command " << trimmedCommand << " Finished";
            finReceived = true;
        }

        // If both ACK and FIN are received, exit the loop
        if (ackReceived && finReceived) {
            loop.quit(); // Exit the event loop
        }
    });

    // Step 4: Setup a timeout mechanism
    timer.setSingleShot(true); // Timer will only fire once
    timer.start(20000); // Set timeout for 5 seconds

    // Step 5: Connect the timer timeout signal to quit the event loop
    connect(&timer, &QTimer::timeout, &loop, [&]() {
        qDebug() << "Timeout waiting for response";
        loop.quit(); // Exit the event loop
    });

    // Step 6: Start the event loop and wait for either response or timeout!
    loop.exec();

    // Step 7: Check the results and return accordingly
    if (ackReceived && finReceived) {
        return 1; // Both responses received
    } else {
        return 0; // Timeout or invalid response
    }
}


// Functions for Gripping
bool Grip();
bool Grip(int force);
bool Grip(int force, int size);
// Functions for releasing
bool Release();
bool Release(int pullback);




