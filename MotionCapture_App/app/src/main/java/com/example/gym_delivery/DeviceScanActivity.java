package com.example.gym_delivery;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.ListActivity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

public class DeviceScanActivity extends ListActivity {
    private final static String TAG = DeviceScanActivity.class.getSimpleName();

    public static final String EXTRAS_DEVICE_NAME = "DEVICE_NAME";
    public static final String EXTRAS_DEVICE_ADDRESS = "DEVICE_ADDRESS";

    private LeDeviceListAdapter mLeDeviceListAdapter;
    private BluetoothAdapter mBluetoothAdapter;
    private boolean mScanning;
    private Handler mHandler;

    private static final int REQUEST_ENABLE_BT = 1;
    private Menu menu;
    private static final long SCAN_PERIOD = 10000;      // 10초간 스캔 후 종료
    int user_data_count = 0;                            // 블루투스 기기 개수 카운트
    boolean mScanning_run = false;                      // 스캐닝중인지 확인하는 변수
    String format_alldata;                              // 16진수로 변경한 user_alldata를 넣는 변수
    BLE_Device mBLE_Device;                             // BLE 데이터들을 묶은 BLE_Device클래스의 객체

    final static String foldername = Environment.getExternalStorageDirectory().getAbsolutePath() + "/MotionCapture";      // 로그파일이 저장되는 폴더의 경로
    final static String filename = "BleLogFile.txt";                                                                    // 로그파일의 파일명
    SimpleDateFormat timeformat = new SimpleDateFormat("yyyy:MM:dd-HH:mm:ss");                                  // 로그의 시간 형식
    String logtext;     // 로그파일에 저장되는 데이터
    Date datetime;      // 현재 시간
    String time;        // datatime을 timeformat형식에 맞춰 바꾼 시간

    //BLE 연결 및 블루투스 사용가능 여부 확인
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.e(TAG, "onCreate");

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_device_scan);

        mBLE_Device = new BLE_Device();

        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            Toast.makeText(this, R.string.ble_not_supported, Toast.LENGTH_SHORT).show();
            finish();
        }
        final BluetoothManager bluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, R.string.error_bluetooth_not_supported, Toast.LENGTH_SHORT).show();
            finish();
            return;
        }
    }

    // 상단의 SCAN, CLEAR 메뉴바 활성화
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        Log.e(TAG, "onCreateOptionsMenu");

        this.menu = menu;
        getMenuInflater().inflate(R.menu.device_scan, menu);

        return true;
    }

    // 메뉴아이템이 선택되었을 때 호출되는 함수
    // SCAN, CLEAR 버튼 기능을 정의
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Log.e(TAG, "onOptionsItemSelected");

        switch (item.getItemId()) {
            case R.id.scan:
				Log.e(TAG, "case : scan");

				if (mScanning_run) {
					// STOP을 누르면 실행되며, 블루투스 스캔을 정지합니다
                    scanLeDevice(false);						// 스캔 종료
                    menu.findItem(R.id.scan).setTitle(R.string.actionbar_scan);			// 메뉴의 STOP을 SCAN으로 변환
                    menu.findItem(R.id.clear).setEnabled(true);			// CLEAR 버튼 활성화
                    mScanning_run = false;
                } else {
					// SCAN을 누르면 실행되며, 블루투스 스캔을 시작합니다
					mLeDeviceListAdapter.clear();						// 디바이스 리스트들을 초기화합니다
                    scanLeDevice(true);							// 스캔 시작
                    menu.findItem(R.id.scan).setTitle(R.string.actionbar_stop);			// 메뉴의 SCAN을 STOP으로 변환
                    menu.findItem(R.id.clear).setEnabled(true);			// CLEAR 버튼 활성화
                    mScanning_run = true;
                }
                break;

            case R.id.clear:
				Log.e(TAG, "case : clear");

                // Initializes list view adapter.
                mLeDeviceListAdapter.clear();							// 디바이스 리스트들을 초기화합니다
                mLeDeviceListAdapter = new LeDeviceListAdapter();
                setListAdapter(mLeDeviceListAdapter);
                break;
        }
        return true;
    }

    // 활동이 재개됨 상태에 들어가면 호출되는 함수
    @Override
    protected void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();

        if (!mBluetoothAdapter.isEnabled()) {
        	Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        	startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        // Initializes list view adapter.
        mLeDeviceListAdapter = new LeDeviceListAdapter();
        setListAdapter(mLeDeviceListAdapter);
    }

    // 활동이 소멸되기 전에 호출되는 함수
    @Override
    public void onDestroy() {
        Log.e(TAG, "onDestroy");
        super.onDestroy();
    }

    // 사용자가 활동을 떠나는것을 나타내기 위해 호출되는 함수
    @Override
    protected void onPause() {
        Log.e(TAG, "onPause");
        super.onPause();
        mScanning_run = false;
        scanLeDevice(false);
        menu.findItem(R.id.scan).setTitle(R.string.actionbar_scan);
        menu.findItem(R.id.clear).setEnabled(true);
        mLeDeviceListAdapter.clear();
    }

    // 다른 액티비티를 호출하여 넘어갔다가 돌아올 때 호출되는 함수
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // User chose not to enable Bluetooth.
        Log.e(TAG, "onActivityResult");
        if (requestCode == REQUEST_ENABLE_BT && resultCode == Activity.RESULT_CANCELED) {
            finish();
            return;
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    // 리스트의 아이템을 클릭했을 때 실행되는 함수
    @Override
    protected void onListItemClick(ListView l, View v, int position, long id) {
    }

    //BLE Scanning 함수
    private void scanLeDevice(final boolean enable) {
        user_data_count = 0;
        Log.e(TAG, "scanLeDevice:" + enable);
        if (enable) {
            mScanning = true;
            mBluetoothAdapter.startLeScan(mLeScanCallback);
        } else {
            mScanning = false;
            mBluetoothAdapter.stopLeScan(mLeScanCallback);
        }
    }

    // 스캔을 통해 찾은 장치들을 고정하기 위한 어댑터입니다
    private class LeDeviceListAdapter extends BaseAdapter {
        private ArrayList<BluetoothDevice> mLeDevices;
        private LayoutInflater mInflator;

        // LeDeviceListAdapter 생성자
        public LeDeviceListAdapter() {
            super();
            mLeDevices = new ArrayList<BluetoothDevice>();
            mInflator = DeviceScanActivity.this.getLayoutInflater();
        }

        // 블루투스 장치가 목록에 없다면 추가합니다
        public void addDevice(BluetoothDevice device) {
            if (!mLeDevices.contains(device)) {
                    mLeDevices.add(device);
            }
        }

        public BluetoothDevice getDevice(int position) {
            return mLeDevices.get(position);
        }

        public void clear() { mLeDevices.clear(); }

        @Override
        public int getCount() {
            return mLeDevices.size();
        }
        @Override
        public Object getItem(int i) {
            return mLeDevices.get(i);
        }
        @Override
        public long getItemId(int i) {
            return i;
        }

        @SuppressLint("InflateParams")
        @Override
        public View getView(int i, View view, ViewGroup parent) {
            ViewHolder viewHolder;
            try {
                // General ListView optimization code.
                if (view == null) {
                    view = mInflator.inflate(R.layout.listview_item, null);

                    viewHolder = new ViewHolder();
                    viewHolder.deviceName = (TextView) view.findViewById(R.id.device_name);
                    viewHolder.deviceAddress = (TextView) view.findViewById(R.id.device_address);
                    viewHolder.textView_rssi = (TextView) view.findViewById(R.id.textView_rssi);
                    viewHolder.textView_user_data = (TextView) view.findViewById(R.id.textView_user_data);
                    viewHolder.textView_user_all_data = (TextView) view.findViewById(R.id.textView_user_all_data);
                    viewHolder.deviceUUID = (TextView) view.findViewById(R.id.device_uuid);

                    view.setTag(viewHolder);
                } else {
                    viewHolder = (ViewHolder) view.getTag();
                }

                BluetoothDevice device = mLeDevices.get(i);

                // 디바이스 이름 설정
                final String deviceName = device.getName();
                if (deviceName != null && deviceName.length() > 0) { viewHolder.deviceName.setText("Name: " + deviceName); }
                else { viewHolder.deviceName.setText(R.string.unknown_device); }
                // 디바이스 주소 설정
                viewHolder.deviceAddress.setText("Address: " + device.getAddress());
                // 디바이스 UUID 설정
                viewHolder.deviceUUID.setText("UUID : " + device.getUuids());
                // 디바이스 Rssi 설정
                viewHolder.textView_rssi.setText("RSSI: " + Integer.toString(mBLE_Device.rssi[i]) + "dBm");
                // 디바이스 all_data 설정
                format_alldata = stringtohex(mBLE_Device.user_alldata[i]);
                viewHolder.textView_user_all_data.setText(format_alldata);

                if(device.getAddress().equals("8C:AA:B5:85:AC:02"))
                {
                    datetime = new Date();                      // 현재 시간을 저장
                    time = timeformat.format(datetime);         // 저장한 시각을 미리 지정한 포맷으로 변환
                    logtext = time + "-" + format_alldata;      // logtext에 "현재시간-데이터" 저장

                    WriteTextFile(foldername, filename, logtext + "\n");
                }

            } catch (Exception e) {
            }

            return view;
        }
    }

    // Device scan callback.
    private BluetoothAdapter.LeScanCallback mLeScanCallback =
            new BluetoothAdapter.LeScanCallback() {
                @Override
                public void onLeScan(final BluetoothDevice device, final int rssi, final byte[] scanRecord) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {

                            mLeDeviceListAdapter.addDevice(device);
                            mLeDeviceListAdapter.notifyDataSetChanged();

                            for (int i = 0; i < mLeDeviceListAdapter.getCount() + 1; i++) {
                                String address = mLeDeviceListAdapter.getDevice(i).getAddress();

                                if (device.getAddress().equals(address)) {
                                    user_data_count = i;
                                    break;
                                }
                            }

                            mBLE_Device.rssi[user_data_count] = rssi;
                            parseAdvertisementPacket(scanRecord);
                        }

                        void parseAdvertisementPacket(final byte[] scanRecord) {

                            byte[] advertisedData = Arrays.copyOfRange(scanRecord, 0, scanRecord.length);
                            final StringBuilder stringBuilder = new StringBuilder(advertisedData.length);
                            for (byte byteChar : advertisedData) {
                                stringBuilder.append((char) byteChar);
                            }

                            mBLE_Device.user_alldata[user_data_count] = stringBuilder.toString();

                        }
                    });
                }
            };

    static class ViewHolder {
        TextView deviceName;
        TextView deviceAddress;
        TextView textView_rssi;
        TextView textView_user_data;
        TextView textView_user_all_data;
        TextView deviceUUID;
    }

    private static String stringtohex(String s) {
        String result = "";

        for (int i = 0; i < s.length(); i++) {
            result += String.format("%02X ", (int) s.charAt(i));
        }
        return result;
    }

    static class BLE_Device {
        String[] user_alldata = new String[50];
        String[] user_data = new String[50];
        String[] user_TX_level = new String[50];
        String[] user_uuid = new String[50];
        int[] rssi = new int[50];

        public void setclear() {
            for (int i = 0; i < 50; i++) {
                user_alldata[i] = "";
                user_data[i] = "";
                user_TX_level[i] = "";
                user_uuid[i] = "";
                rssi[i] = 0;
            }
        }
    }

    //텍스트내용을 경로의 텍스트 파일에 쓰기
    public void WriteTextFile(String foldername, String filename, String contents){
        Log.e(TAG,"로그파일생성");
        try{
            File dir = new File (foldername);
            //디렉토리 폴더가 없으면 생성함
            if(!dir.exists()){
                dir.mkdir();
            }
            //파일 output stream 생성
            FileOutputStream fos = new FileOutputStream(foldername+"/"+filename, true);
            //파일쓰기
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(fos));
            writer.append(contents);
            //writer.write(contents);
            writer.flush();

            writer.close();
            fos.close();
        }catch (IOException e){
            e.printStackTrace();
        }
        Log.e(TAG,"로그파일쓰기 끝");

    }

}
