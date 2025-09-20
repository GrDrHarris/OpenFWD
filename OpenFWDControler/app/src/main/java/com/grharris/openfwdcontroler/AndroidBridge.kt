package com.grharris.openfwdcontroler

import android.util.Log
import android.webkit.WebView
import java.io.IOException
import java.lang.NumberFormatException
import java.net.InetSocketAddress
import java.net.Socket
import java.net.UnknownHostException

class AndroidBridge(
    private val context: android.content.Context,
    private val webView: WebView
) {
    private var connection: Socket = Socket()
    @android.webkit.JavascriptInterface
    fun connectPeer(host: String, port: String): String {
        try {
            if(connection.isClosed)
                connection = Socket()
            val portInt = port.toInt()
            connection.connect(InetSocketAddress(host, portInt), 1000)
            return ""
        } catch (e : NumberFormatException) {
            Log.w("WARN",  e.message ?: "Unknown error")
            return "Invalid port"
        } catch (e : UnknownHostException) {
            Log.w("WARN",  e.message ?: "Unknown error")
            return "Unknown host"
        } catch (e : IOException) {
            Log.w("WARN",  e.message ?: "Unknown error")
            return "Failed to connect"
        } catch (e : SecurityException) {
            Log.w("WARN",  e.message ?: "Unknown error")
            return "Security exception"
        } catch (e : IllegalArgumentException) {
            Log.w("WARN",  e.message ?: "Unknown error")
            return "Invalid port"
        }
    }

    @android.webkit.JavascriptInterface
    fun sendUdpPacket(host: String, port: Int, message: String) {
        Thread {
            try {
                java.net.DatagramSocket().use { socket ->
                    val address = java.net.InetAddress.getByName(host)
                    val packet = java.net.DatagramPacket(
                        message.toByteArray(),
                        message.length,
                        address,
                        port
                    )
                    socket.send(packet)

                    webView.post {
                        webView.evaluateJavascript(
                            "window.onUdpSent(true)",
                            null
                        )
                    }
                }
            } catch (e: Exception) {
                webView.post {
                    webView.evaluateJavascript(
                        "window.onUdpError('${e.message?.escapeJs()}')",
                        null
                    )
                }
            }
        }.start()
    }

    // 防止JS注入攻击的转义
    private fun String.escapeJs(): String {
        return this.replace("'", "\\'")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
    }
}