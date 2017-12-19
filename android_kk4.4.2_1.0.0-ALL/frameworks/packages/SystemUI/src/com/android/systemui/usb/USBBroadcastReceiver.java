package com.android.systemui.usb;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbManager;
import android.provider.Settings;
import android.util.Log;


public class USBBroadcastReceiver extends BroadcastReceiver {
    private static final String TAG = "SystemUI_USB_Receiver";
    private static final String USB_CONNECT = "com.iline.ihip.usb_connect";
    private static final String USB_UNCONNECT = "com.iline.ihip.usb_unconnect";
    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent.getAction().equals(UsbManager.ACTION_USB_STATE)) {
            if (intent.getExtras().getBoolean(UsbManager.USB_CONNECTED)) {
                Log.d(TAG, "onReceive: usb-connect");
                Settings.System.putInt(context.getContentResolver(), Settings.System.USB_CONNECT_STATUS, 1);
                context.sendBroadcast(new Intent(USB_CONNECT));
            } else {
                Log.d(TAG, "onReceive: usb-unconnect");
                Settings.System.putInt(context.getContentResolver(), Settings.System.USB_CONNECT_STATUS, 0);
                context.sendBroadcast(new Intent(USB_UNCONNECT));
            }
        }
    }
}
