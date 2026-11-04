#Requires -Version 5.1
<#
.SYNOPSIS
    把本仓库目录一键发布到 GitHub（直接走 REST API，本机不需要安装 git）。

.DESCRIPTION
    流程：校验 Token → 查账号 → 仓库不存在就创建 → 每个文件上传为 blob
          → 建 tree + commit → 更新分支 → 打印仓库地址。
    适用于本机访问 github.com 主站不稳定、但 api.github.com 可达的情况。

.PARAMETER Token
    GitHub Personal Access Token（classic 需 repo 权限；fine-grained 需
    "Administration: read/write" + "Contents: read/write"）。默认读 $env:GITHUB_TOKEN。

.PARAMETER Owner
    账号或组织名。留空则用 Token 对应的登录账号。

.PARAMETER Repo
    仓库名，默认 zangmaxiong。

.PARAMETER Private
    建为私有仓库（默认公开）。

.EXAMPLE
    $env:GITHUB_TOKEN = 'ghp_xxxxxxxx'
    .\tools\publish-to-github.ps1 -Owner caolvchong66 -Repo zangmaxiong

.EXAMPLE
    .\tools\publish-to-github.ps1 -DryRun      # 只列文件，不联网
#>
[CmdletBinding()]
param(
    [string]$Token = $env:GITHUB_TOKEN,
    [string]$Owner,
    [string]$Repo = 'zangmaxiong',
    [string]$Description = '基于 ZYNQ-7020 的藏马熊识别与预警装置：OV5640 采集 + PL 零拷贝显示 + 自研 INT8 推理引擎运行 YOLOv8n，裸机实时检测、画框与蜂鸣器报警',
    [switch]$Private,
    [string]$Branch = 'main',
    [string]$ApiBase = 'https://api.github.com',
    [string]$Path = '',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
try { [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12 } catch { }

# param() 默认值求值时 $PSScriptRoot 还不可用，放到这里算
if ([string]::IsNullOrWhiteSpace($Path)) { $Path = Split-Path -Parent $PSScriptRoot }
$Path = (Resolve-Path -LiteralPath $Path).Path.TrimEnd('\')

# ---------------------------------------------------------------- 收集文件
$files = @(
    Get-ChildItem -LiteralPath $Path -Recurse -File -Force |
        Where-Object { $_.FullName -notmatch '\\\.git\\' } |
        ForEach-Object {
            [pscustomobject]@{
                Full   = $_.FullName
                Rel    = $_.FullName.Substring($Path.Length + 1).Replace('\', '/')
                Length = $_.Length
            }
        } | Sort-Object Length
)

$totalMB = [math]::Round((($files | Measure-Object Length -Sum).Sum) / 1MB, 2)
Write-Host "待发布：$($files.Count) 个文件，合计 $totalMB MB（$Path）" -ForegroundColor Cyan

$tooBig = $files | Where-Object { $_.Length -gt 100MB }
if ($tooBig) {
    throw "以下文件超过 GitHub 单文件 100 MB 限制，请先处理：`n" + (($tooBig | ForEach-Object { "  $($_.Rel)  $([math]::Round($_.Length/1MB,1)) MB" }) -join "`n")
}

if ($DryRun) {
    $files | ForEach-Object { "{0,10:N1} KB  {1}" -f ($_.Length / 1KB), $_.Rel }
    Write-Host "`n[DryRun] 未联网，未做任何修改。" -ForegroundColor Yellow
    return
}

# ---------------------------------------------------------------- API 封装
if ([string]::IsNullOrWhiteSpace($Token)) {
    throw "缺少 GitHub Token。请先执行 `$env:GITHUB_TOKEN = 'ghp_...'，或用 -Token 传入。"
}

function Invoke-Api {
    param(
        [Parameter(Mandatory)][string]$Method,
        [Parameter(Mandatory)][string]$Url,
        $Body,
        [int[]]$OkStatus = @()
    )
    $params = @{
        Method     = $Method
        Uri        = $Url
        Headers    = @{
            Authorization          = "Bearer $Token"
            Accept                 = 'application/vnd.github+json'
            'User-Agent'           = 'bear-warning-device-publisher'
            'X-GitHub-Api-Version' = '2022-11-28'
        }
        TimeoutSec = 900
    }
    if ($PSBoundParameters.ContainsKey('Body') -and $null -ne $Body) {
        $json = $Body | ConvertTo-Json -Depth 12 -Compress
        $params['Body'] = [System.Text.Encoding]::UTF8.GetBytes($json)
        $params['ContentType'] = 'application/json; charset=utf-8'
    }
    try {
        return Invoke-RestMethod @params
    }
    catch {
        $resp = $_.Exception.Response
        $code = 0
        if ($resp) { $code = [int]$resp.StatusCode }
        if ($OkStatus -contains $code) { return $null }
        $detail = ''
        if ($resp) {
            try {
                $sr = New-Object System.IO.StreamReader($resp.GetResponseStream())
                $detail = $sr.ReadToEnd()
                $sr.Dispose()
            } catch { }
        }
        throw "GitHub API 调用失败：$Method $Url`nHTTP $code`n$detail`n$($_.Exception.Message)"
    }
}

function ConvertTo-Base64([string]$file) {
    [Convert]::ToBase64String([System.IO.File]::ReadAllBytes($file))
}

# ---------------------------------------------------------------- 1. 账号
$who = Invoke-Api -Method GET -Url "$ApiBase/user"
if ([string]::IsNullOrWhiteSpace($Owner)) { $Owner = $who.login }
Write-Host "账号：$($who.login)   目标仓库：$Owner/$Repo" -ForegroundColor Cyan

# ---------------------------------------------------------------- 2. 仓库
$repoUrl = "$ApiBase/repos/$Owner/$Repo"
$repoInfo = Invoke-Api -Method GET -Url $repoUrl -OkStatus @(404)

if ($repoInfo) {
    Write-Host "仓库已存在，将向其中追加/覆盖文件。" -ForegroundColor Yellow
}
else {
    $createUrl = if ($Owner -eq $who.login) { "$ApiBase/user/repos" } else { "$ApiBase/orgs/$Owner/repos" }
    $payload = @{
        name        = $Repo
        description = $Description
        private     = [bool]$Private
        has_issues  = $true
        has_wiki    = $false
        has_projects= $false
        auto_init   = $false
    }
    $repoInfo = Invoke-Api -Method POST -Url $createUrl -Body $payload
    Write-Host "已创建$(if ($Private) {'私有'} else {'公开'})仓库 $($repoInfo.full_name)" -ForegroundColor Green
}

# ---------------------------------------------------------------- 3. 分支头
$headSha = $null
$baseTree = $null
$refName = $Branch
$refInfo = Invoke-Api -Method GET -Url "$repoUrl/git/ref/heads/$refName" -OkStatus @(404, 409)

if (-not $refInfo) {
    # 空仓库：GitHub 的 Git Data API 要求仓库至少有一次提交，否则创建 blob 会返回 409。
    # 先用 Contents API 打一次底（临时文件随后会被本次上传覆盖）。
    Write-Host "仓库还是空的，先创建初始提交 ..." -ForegroundColor Yellow
    $seedB64 = [Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes("# placeholder`n"))
    Invoke-Api -Method PUT -Url "$repoUrl/contents/.gitattributes" -Body @{
        message = 'chore: 初始化仓库'
        content = $seedB64
        branch  = $refName
    } | Out-Null
    Start-Sleep -Seconds 2
    $refInfo = Invoke-Api -Method GET -Url "$repoUrl/git/ref/heads/$refName"
}

if ($refInfo) {
    $headSha = $refInfo.object.sha
    $commit = Invoke-Api -Method GET -Url "$repoUrl/git/commits/$headSha"
    $baseTree = $commit.tree.sha
    Write-Host "基于已有提交：$($headSha.Substring(0,7))" -ForegroundColor DarkGray
}

# ---------------------------------------------------------------- 4. blobs
$entries = New-Object System.Collections.ArrayList
$i = 0
foreach ($f in $files) {
    $i++
    $pct = [int](100 * $i / $files.Count)
    Write-Progress -Activity "上传文件到 $Owner/$Repo" -Status "[$i/$($files.Count)] $($f.Rel)" -PercentComplete $pct
    Write-Host ("  [{0,3}/{1}] {2}  ({3:N1} KB)" -f $i, $files.Count, $f.Rel, ($f.Length / 1KB)) -ForegroundColor DarkGray

    $blob = Invoke-Api -Method POST -Url "$repoUrl/git/blobs" -Body @{
        content  = (ConvertTo-Base64 $f.Full)
        encoding = 'base64'
    }
    [void]$entries.Add(@{
        path = $f.Rel
        mode = '100644'
        type = 'blob'
        sha  = $blob.sha
    })
}
Write-Progress -Activity "上传文件到 $Owner/$Repo" -Completed

# ---------------------------------------------------------------- 5. tree
Write-Host "生成目录树 ..." -ForegroundColor Cyan
$treeBody = @{ tree = @($entries) }
if ($baseTree) { $treeBody['base_tree'] = $baseTree }
$tree = Invoke-Api -Method POST -Url "$repoUrl/git/trees" -Body $treeBody

# ---------------------------------------------------------------- 6. commit
$msg = "藏马熊识别与预警装置：工程源码、INT8 模型、启动镜像与文档`n`n由 U 盘原始资料整理发布。"
$commitBody = @{
    message = $msg
    tree    = $tree.sha
}
if ($headSha) { $commitBody['parents'] = @($headSha) }
$commit2 = Invoke-Api -Method POST -Url "$repoUrl/git/commits" -Body $commitBody

# ---------------------------------------------------------------- 7. ref
if ($headSha) {
    Invoke-Api -Method PATCH -Url "$repoUrl/git/refs/heads/$refName" -Body @{ sha = $commit2.sha; force = $true } | Out-Null
}
else {
    Invoke-Api -Method POST -Url "$repoUrl/git/refs" -Body @{ ref = "refs/heads/$refName"; sha = $commit2.sha } | Out-Null
}

# 新建的空仓库，确保默认分支是我们推上去的分支
try {
    Invoke-Api -Method PATCH -Url $repoUrl -Body @{ default_branch = $refName } | Out-Null
} catch { }

# ---------------------------------------------------------------- 8. 完成
$html = "https://github.com/$Owner/$Repo"
Write-Host ""
Write-Host "==================================================" -ForegroundColor Green
Write-Host " 发布完成" -ForegroundColor Green
Write-Host " 仓库地址：$html" -ForegroundColor Green
Write-Host " 提交：$($commit2.sha.Substring(0,7))   文件：$($files.Count) 个 / $totalMB MB" -ForegroundColor Green
Write-Host "==================================================" -ForegroundColor Green
