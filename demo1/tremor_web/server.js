// server.js - 本地代理，把前端请求转发到 LM Studio

const express = require("express");
const fetch = require("node-fetch");   // 使用 node-fetch@2

const app = express();
const PORT = 3000;

// 解析 JSON 请求体
app.use(express.json());

// 静态文件服务：让你可以直接访问当前目录下的 html/js 等文件
app.use(express.static("."));

// 前端调用 /api/diagnose，让这个代理再去请求 LM Studio
app.post("/api/diagnose", async (req, res) => {
  console.log(">>> /api/diagnose called");
  // 打印一下请求体长度
  console.log("payload size:", JSON.stringify(req.body).length);
  try {
    const payload = req.body;

    const lmResp = await fetch("http://127.0.0.1:1234/v1/chat/completions", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(payload)
    });

    const text = await lmResp.text();
    res.status(lmResp.status).type("application/json").send(text);
  } catch (err) {
    console.error("Proxy error:", err);
    res.status(500).json({ error: err.message });
  }
});

app.listen(PORT, () => {
  console.log(`Server running at http://127.0.0.1:${PORT}`);
  console.log(`Open:  http://127.0.0.1:${PORT}/tremor_monitor.html`);
});
