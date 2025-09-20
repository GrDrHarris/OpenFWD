package com.grharris.openfwdcontroler

import android.os.Build
import android.os.Bundle
import android.util.Log
import android.webkit.WebView
import android.widget.Toast
import androidx.activity.ComponentActivity
import java.io.File
import java.io.IOException

class MainActivity : ComponentActivity() {
    private lateinit var webView: WebView
    private lateinit var webRootDir: File

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // 初始化WebView
        webView = findViewById(R.id.webView)
        setupWebView()

        // 初始化网页根目录（可读写）
        webRootDir = File(filesDir, "web")
        //if (!webRootDir.exists()) {
            copyAssetsToWebDir() // 首次运行拷贝assets中的默认网页
        //}

        // 启动本地HTTP服务器（端口8080）
        try {
            webView.loadUrl("file://${webRootDir.path}/index.html")
        } catch (e: IOException) {
            Toast.makeText(this, "无法启动本地服务器", Toast.LENGTH_SHORT).show()
            e.printStackTrace()
        }
    }

    private fun setupWebView() {
        // 启用JavaScript
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.CUPCAKE) {
            webView.settings.allowFileAccess = true
        }
        webView.settings.javaScriptEnabled = true

        // 暴露AndroidBridge给JS
        webView.addJavascriptInterface(
            AndroidBridge(this, webView),
            "AndroidBridge" // JS通过window.AndroidBridge访问
        )

        // 允许调试（可选）
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT) {
            WebView.setWebContentsDebuggingEnabled(true)
        }
    }

    /**
     * 从assets拷贝默认网页到可读写目录
     */
    private fun copyAssetsToWebDir() {
        webRootDir.mkdirs()
        assets.list("web")?.forEach { filename ->
            assets.open("web/$filename").use { input ->
                File(webRootDir, filename).outputStream().use { output ->
                    input.copyTo(output)
                }
            }
        }
    }

    /**
     * 更新网页内容（通过下载ZIP包）
     */
//    private fun updateWebContent(zipUrl: String) {
//        // 实际项目中应使用协程或RxJava处理异步
//        Thread {
//            try {
//                val zipFile = File(cacheDir, "update.zip")
//                downloadFile(zipUrl, zipFile) // 实现你的下载逻辑
//                unzipFile(zipFile, webRootDir) // 实现解压逻辑
//
//                runOnUiThread {
//                    webView.reload()
//                    Toast.makeText(this, "网页更新成功", Toast.LENGTH_SHORT).show()
//                }
//            } catch (e: Exception) {
//                runOnUiThread {
//                    Toast.makeText(this, "更新失败: ${e.message}", Toast.LENGTH_SHORT).show()
//                }
//            }
//        }.start()
//    }

    override fun onDestroy() {
        super.onDestroy()
    }
}