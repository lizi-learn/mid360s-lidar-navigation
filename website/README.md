# NavBridge 文档站

七个静态页面：概览、快速接入、完整项目手册、API、兼容矩阵、合成交互演示和下载。正文来自 `docs/sdk/`，接口来自 OpenAPI；没有硬编码的真实机器人控制地址或浏览器凭据。

## 构建

Python 3.10+、Node18+。建议使用独立 venv，以免系统 setuptools 过旧或系统 PYTHONPATH 覆盖构建环境。

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install 'setuptools>=65' wheel -r requirements-sdk-docs.txt
python -m pip wheel --no-build-isolation --no-deps . -w dist
(cd clients/typescript && npm pack --ignore-scripts --pack-destination ../../dist)
python scripts/build-sdk-openapi.py
python scripts/build-sdk-pdf.py
python scripts/build-sdk-site.py
python -m http.server 18942 --bind 127.0.0.1 --directory website/dist
```

PDF渲染器默认使用系统 `fonts-wqy-zenhei` 与 DejaVu Mono。手册生成后需实际渲染检查。`website/dist/`、`dist/`、`output/`为构建产物；不将私有运行目录或第三方厂商手册复制到站点。

## Cloudflare Pages

采用 Direct Upload 静态项目，构建输出为 `website/dist/`，不需要 Functions 或 VPS。将 Pages 的实际返回域名绑定到自己的自定义域名。部署凭据仅放在发布者本机/CI秘密中，仓库不包含账户密钥或本机凭据文件。

```bash
wrangler pages deploy website/dist --project-name YOUR_PAGES_PROJECT --branch main
```

先检查目标项目和DNS是否属于本次站点，避免覆盖其他产品站。部署后检查主页、关键页面、实际下载与SHA256；浏览器检查桌面/手机布局、搜索、代码页签、API筛选和演示开始/暂停/等待继续。

合成演示是明确编写的交互路线，不代表Nav2算法、办公室地图、双足动力学或真机验收。
