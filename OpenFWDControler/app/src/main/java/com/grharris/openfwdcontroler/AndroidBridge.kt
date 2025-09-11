package com.grharris.openfwdcontroler

import android.webkit.WebView

class AndroidBridge(
    private val context: android.content.Context,
    private val webView: WebView
) {

    @android.webkit.JavascriptInterface
    fun sendTcpRequest(host: String, port: Int, data: String) {
        Thread {
            try {
                java.net.Socket(host, port).use { socket ->
                    socket.getOutputStream().write(data.toByteArray())

                    // 读取响应（示例）
                    val input = socket.getInputStream()
                    val response = input.bufferedReader().readText()

                    webView.post {
                        webView.evaluateJavascript(
                            "window.onTcpResponse('${response.escapeJs()}')",
                            null
                        )
                    }
                }
            } catch (e: Exception) {
                webView.post {
                    webView.evaluateJavascript(
                        "window.onTcpError('${e.message?.escapeJs()}')",
                        null
                    )
                }
            }
        }.start()
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