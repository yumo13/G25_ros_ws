#!/usr/bin/env node

/**
 * Node.js BLE Server using @abandonware/bleno
 * iOSアプリからの2つのfloat値を受信し、ROS2ノードにデータを送信
 */

const bleno = require('@abandonware/bleno');
const fs = require('fs');
const path = require('path');

// UUIDs
const SERVICE_UUID = '12345678-1234-1234-1234-123456789abc';
const CHARACTERISTIC_UUID = '87654321-4321-4321-4321-cba987654321';

// データ転送用のパイプファイルパス
const PIPE_PATH = '/tmp/ble_ros_pipe';

/**
 * カスタムキャラクタリスティック
 */
class CmdVelCharacteristic extends bleno.Characteristic {
    constructor() {
        super({
            uuid: CHARACTERISTIC_UUID,
            properties: ['write'],
            value: null
        });
        
        console.log('CmdVelCharacteristic initialized');
        this.setupPipe();
    }
    
    /**
     * 名前付きパイプの設定
     */
    setupPipe() {
        try {
            // 既存のパイプを削除
            if (fs.existsSync(PIPE_PATH)) {
                fs.unlinkSync(PIPE_PATH);
            }
            
            // 名前付きパイプを作成
            require('child_process').execSync(`mkfifo ${PIPE_PATH}`);
            // パイプファイルの権限を666に設定
            require('child_process').execSync(`chmod 666 ${PIPE_PATH}`);
            console.log(`Created named pipe: ${PIPE_PATH} with permissions 666`);
            
        } catch (error) {
            console.error('Failed to create pipe:', error.message);
        }
    }
    
    /**
     * 書き込み要求の処理
     */
    onWriteRequest(data, offset, withoutResponse, callback) {
        console.log(`Received ${data.length} bytes:`, data);
        
        if (data.length === 8) { // 2つのfloat値（4バイト×2）
            try {
                // リトルエンディアンで2つのfloat値を読み取り
                const linear_y = data.readFloatLE(0);
                const angular_z = data.readFloatLE(4);
                
                console.log(`Unpacked values: linear.y=${linear_y}, angular.z=${angular_z}`);
                
                // ROS2ノードにデータを送信
                this.sendToRos2(linear_y, angular_z);
                
                // 成功をコールバック
                callback(this.RESULT_SUCCESS);
                
            } catch (error) {
                console.error('Failed to process data:', error.message);
                callback(this.RESULT_INVALID_ATTRIBUTE_LENGTH);
            }
        } else {
            console.warn(`Invalid data length: ${data.length} bytes (expected 8)`);
            callback(this.RESULT_INVALID_ATTRIBUTE_LENGTH);
        }
    }
    
    /**
     * ROS2ノードにデータを送信
     */
    sendToRos2(linear_y, angular_z) {
        try {
            const message = JSON.stringify({
                linear_y: linear_y,
                angular_z: angular_z,
                timestamp: Date.now()
            });
            
            // 名前付きパイプにデータを書き込み
            // パイプが存在しない場合はスキップ
            if (!fs.existsSync(PIPE_PATH)) {
                console.warn('Pipe does not exist, skipping data send');
                return;
            }
            
            // 直接書き込み（同期的）
            try {
                fs.appendFileSync(PIPE_PATH, message + '\n');
                console.log('Data sent to ROS2 node:', message);
            } catch (writeError) {
                if (writeError.code === 'EPIPE' || writeError.code === 'ENOENT') {
                    console.warn('Pipe not ready for writing, data skipped');
                } else {
                    console.error('Failed to write to pipe:', writeError.message);
                }
            }
            
        } catch (error) {
            console.error('Failed to send data to ROS2:', error.message);
        }
    }
}

/**
 * BLEサーバークラス
 */
class BleServer {
    constructor() {
        this.primaryService = null;
        this.cmdVelChar = new CmdVelCharacteristic();
        
        // BLEイベントハンドラーの設定
        bleno.on('stateChange', this.onStateChange.bind(this));
        bleno.on('advertisingStart', this.onAdvertisingStart.bind(this));
        bleno.on('accept', this.onAccept.bind(this));
        bleno.on('disconnect', this.onDisconnect.bind(this));
        
        console.log('BLE Server initialized');
    }
    
    /**
     * BLE状態変更時の処理
     */
    onStateChange(state) {
        console.log(`BLE state changed to: ${state}`);
        
        if (state === 'poweredOn') {
            console.log('Starting advertising...');
            bleno.startAdvertising('NavRobot', [SERVICE_UUID]);
        } else {
            console.log('Stopping advertising...');
            bleno.stopAdvertising();
        }
    }
    
    /**
     * アドバタイジング開始時の処理
     */
    onAdvertisingStart(error) {
        if (!error) {
            console.log('Advertising started successfully!');
            console.log('Setting up services...');
            
            this.primaryService = new bleno.PrimaryService({
                uuid: SERVICE_UUID,
                characteristics: [this.cmdVelChar]
            });
            
            bleno.setServices([this.primaryService], (error) => {
                if (!error) {
                    console.log('Services set up successfully!');
                    console.log(`Service UUID: ${SERVICE_UUID}`);
                    console.log(`Characteristic UUID: ${CHARACTERISTIC_UUID}`);
                } else {
                    console.error('Failed to set up services:', error);
                }
            });
        } else {
            console.error('Advertising start error:', error);
        }
    }
    
    /**
     * クライアント接続時の処理
     */
    onAccept(clientAddress) {
        console.log(`Client connected: ${clientAddress}`);
    }
    
    /**
     * クライアント切断時の処理
     */
    onDisconnect(clientAddress) {
        console.log(`Client disconnected: ${clientAddress}`);
    }
    
    /**
     * サーバー開始
     */
    start() {
        console.log('Starting BLE server...');
        console.log('Press Ctrl+C to stop');
        
        // 終了処理
        process.on('SIGINT', () => {
            console.log('\nStopping BLE server...');
            bleno.stopAdvertising();
            
            // パイプファイルを削除
            if (fs.existsSync(PIPE_PATH)) {
                fs.unlinkSync(PIPE_PATH);
                console.log('Cleaned up pipe file');
            }
            
            process.exit(0);
        });
    }
}

/**
 * メイン実行部
 */
function main() {
    console.log('=== Node.js BLE Server for ROS2 ===');
    console.log(`Service UUID: ${SERVICE_UUID}`);
    console.log(`Characteristic UUID: ${CHARACTERISTIC_UUID}`);
    console.log(`Pipe path: ${PIPE_PATH}`);
    console.log();
    
    const server = new BleServer();
    server.start();
}

// 実行
if (require.main === module) {
    main();
}