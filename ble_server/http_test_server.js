#!/usr/bin/env node

/**
 * 簡易テストサーバー - BLEの代わりにHTTPでテスト
 * 本来のBLE機能の代替として使用
 */

const http = require('http');
const fs = require('fs');

// パイプファイルパス
const PIPE_PATH = '/tmp/ble_ros_pipe';

/**
 * パイプファイルの設定
 */
function setupPipe() {
    try {
        // 既存のパイプを削除
        if (fs.existsSync(PIPE_PATH)) {
            fs.unlinkSync(PIPE_PATH);
        }
        
        // 名前付きパイプを作成
        require('child_process').execSync(`mkfifo ${PIPE_PATH}`);
        console.log(`Created named pipe: ${PIPE_PATH}`);
        
    } catch (error) {
        console.error('Failed to create pipe:', error.message);
    }
}

/**
 * ROS2ノードにデータを送信
 */
function sendToRos2(linear_y, angular_z) {
    try {
        const message = JSON.stringify({
            linear_y: linear_y,
            angular_z: angular_z,
            timestamp: Date.now()
        });
        
        // 名前付きパイプにデータを書き込み
        fs.writeFile(PIPE_PATH, message + '\n', (err) => {
            if (err && err.code !== 'EPIPE') {
                console.error('Failed to write to pipe:', err.message);
            } else {
                console.log('Data sent to ROS2 node:', message);
            }
        });
        
    } catch (error) {
        console.error('Failed to send data to ROS2:', error.message);
    }
}

/**
 * HTTPサーバーの作成
 */
function createHttpServer() {
    const server = http.createServer((req, res) => {
        // CORSヘッダーを設定
        res.setHeader('Access-Control-Allow-Origin', '*');
        res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
        res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

        if (req.method === 'OPTIONS') {
            res.writeHead(200);
            res.end();
            return;
        }

        if (req.method === 'POST' && req.url === '/cmd_vel') {
            let body = '';
            
            req.on('data', (chunk) => {
                body += chunk.toString();
            });
            
            req.on('end', () => {
                try {
                    const data = JSON.parse(body);
                    const linear_y = parseFloat(data.linear_y || 0);
                    const angular_z = parseFloat(data.angular_z || 0);
                    
                    console.log(`Received HTTP request: linear.y=${linear_y}, angular.z=${angular_z}`);
                    
                    // ROS2に送信
                    sendToRos2(linear_y, angular_z);
                    
                    res.writeHead(200, { 'Content-Type': 'application/json' });
                    res.end(JSON.stringify({ status: 'success' }));
                    
                } catch (error) {
                    console.error('Error processing request:', error.message);
                    res.writeHead(400, { 'Content-Type': 'application/json' });
                    res.end(JSON.stringify({ error: 'Invalid JSON' }));
                }
            });
        } else if (req.method === 'GET' && req.url === '/') {
            // 簡単なテストページを提供
            const html = `
<!DOCTYPE html>
<html>
<head>
    <title>BLE Test Server</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .container { max-width: 400px; }
        input { margin: 5px 0; padding: 5px; width: 100px; }
        button { margin: 5px; padding: 10px 20px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ROS2 CMD_VEL Test</h1>
        <div>
            <label>Linear Y: <input type="number" id="linear_y" value="0" step="0.1"></label>
        </div>
        <div>
            <label>Angular Z: <input type="number" id="angular_z" value="0" step="0.1"></label>
        </div>
        <button onclick="sendCommand()">Send Command</button>
        <div id="status"></div>
    </div>
    
    <script>
        function sendCommand() {
            const linear_y = parseFloat(document.getElementById('linear_y').value);
            const angular_z = parseFloat(document.getElementById('angular_z').value);
            
            fetch('/cmd_vel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ linear_y, angular_z })
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerHTML = 
                    '<p style="color: green;">Command sent: linear.y=' + linear_y + ', angular.z=' + angular_z + '</p>';
            })
            .catch(error => {
                document.getElementById('status').innerHTML = 
                    '<p style="color: red;">Error: ' + error + '</p>';
            });
        }
    </script>
</body>
</html>`;
            res.writeHead(200, { 'Content-Type': 'text/html' });
            res.end(html);
        } else {
            res.writeHead(404);
            res.end('Not Found');
        }
    });
    
    return server;
}

/**
 * メイン実行部
 */
function main() {
    console.log('=== HTTP Test Server for ROS2 (BLE Alternative) ===');
    console.log(`Pipe path: ${PIPE_PATH}`);
    
    // パイプ設定
    setupPipe();
    
    // HTTPサーバー開始
    const server = createHttpServer();
    const port = 3000;
    
    server.listen(port, () => {
        console.log(`HTTP server running on http://localhost:${port}`);
        console.log('Open the URL in your browser to test');
        console.log('Or send POST requests to http://localhost:3000/cmd_vel');
        console.log('Press Ctrl+C to stop');
    });
    
    // 終了処理
    process.on('SIGINT', () => {
        console.log('\nStopping HTTP server...');
        server.close();
        
        // パイプファイルを削除
        if (fs.existsSync(PIPE_PATH)) {
            fs.unlinkSync(PIPE_PATH);
            console.log('Cleaned up pipe file');
        }
        
        process.exit(0);
    });
}

// 実行
if (require.main === module) {
    main();
}