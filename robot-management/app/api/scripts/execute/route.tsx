import { type NextRequest, NextResponse } from "next/server"
import { Client } from "ssh2" // 引入 ssh2 库

interface ExecuteRequest {
  server: string
  scriptPath: string
  args?: string
  workingDir?: string
  environmentVars?: Record<string, string>
  timeout?: number
  username?: string // 添加用户名
  password?: string // 添加密码 (不推荐用于生产环境)
  privateKey?: string // 添加私钥 (推荐用于生产环境)
}

interface ExecuteResponse {
  success: boolean
  executionId: string
  message?: string
  error?: string
}

// 模拟的执行状态存储 (在真实后端中，这通常会存储在数据库或缓存中)
const executions = new Map<
  string,
  {
    status: "running" | "completed" | "failed"
    output: string[]
    startTime: Date
    endTime?: Date
    exitCode?: number
  }
>()

export async function POST(request: NextRequest) {
  console.log("=== Script Execution Request Started ===")
  console.log("Timestamp:", new Date().toISOString())

  try {
    const body: ExecuteRequest = await request.json()
    console.log("Request body:", JSON.stringify(body, null, 2))

    const {
      server,
      scriptPath,
      args,
      workingDir,
      environmentVars,
      timeout = 300,
      username,
      password,
      privateKey,
    } = body

    // 验证请求参数
    if (!server || !scriptPath) {
      console.log("❌ Validation failed: Missing required parameters")
      return NextResponse.json({ success: false, error: "Server and script path are required" }, { status: 400 })
    }
    if (!username || (!password && !privateKey)) {
      console.log("❌ Validation failed: Missing SSH credentials (username and password/privateKey)")
      return NextResponse.json(
        { success: false, error: "SSH username and password or privateKey are required" },
        { status: 400 },
      )
    }

    console.log("✅ Request validation passed")
    console.log("Configuration:")
    console.log("  - Server:", server)
    console.log("  - Script path:", scriptPath)
    console.log("  - Arguments:", args || "none")
    console.log("  - Working directory:", workingDir || "default")
    console.log(
      "  - Environment variables:",
      environmentVars ? Object.keys(environmentVars).length + " variables" : "none",
    )
    console.log("  - Timeout:", timeout + "s")
    console.log("  - Username:", username)
    console.log("  - Private Key provided:", !!privateKey)
    console.log("  - Password provided:", !!password)

    // 生成执行ID
    const executionId = `exec_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
    console.log("Generated execution ID:", executionId)

    // 初始化执行状态
    executions.set(executionId, {
      status: "running",
      output: [],
      startTime: new Date(),
    })
    console.log("Execution state initialized")

    // 调用真实的脚本执行函数
    console.log("Calling executeScriptOnRemoteServer (real SSH execution attempt)...")
    executeScriptOnRemoteServer(executionId, {
      server,
      scriptPath,
      args,
      workingDir,
      environmentVars,
      timeout,
      username,
      password,
      privateKey,
    })

    console.log("✅ Script execution initiated successfully")
    console.log("=== Script Execution Request Completed ===\n")

    return NextResponse.json({
      success: true,
      executionId,
      message: "Script execution started",
    })
  } catch (error) {
    console.error("❌ Script execution error:", error)
    console.error("Error stack:", error instanceof Error ? error.stack : "No stack trace")
    console.log("=== Script Execution Request Failed ===\n")
    return NextResponse.json({ success: false, error: "Internal server error" }, { status: 500 })
  }
}

export async function GET(request: NextRequest) {
  const { searchParams } = new URL(request.url)
  const executionId = searchParams.get("executionId")

  console.log("=== Execution Status Query ===")
  console.log("Timestamp:", new Date().toISOString())
  console.log("Execution ID:", executionId)

  if (!executionId) {
    console.log("❌ Missing execution ID")
    return NextResponse.json({ success: false, error: "Execution ID is required" }, { status: 400 })
  }

  const execution = executions.get(executionId)
  if (!execution) {
    console.log("❌ Execution not found for ID:", executionId)
    console.log("Available executions:", Array.from(executions.keys()))
    return NextResponse.json({ success: false, error: "Execution not found" }, { status: 404 })
  }

  console.log("✅ Execution found:")
  console.log("  - Status:", execution.status)
  console.log("  - Output lines:", execution.output.length)
  console.log("  - Start time:", execution.startTime.toISOString())
  console.log("  - End time:", execution.endTime?.toISOString() || "still running")
  console.log("  - Exit code:", execution.exitCode ?? "not available")

  return NextResponse.json({
    success: true,
    execution: {
      status: execution.status,
      output: execution.output,
      startTime: execution.startTime,
      endTime: execution.endTime,
      exitCode: execution.exitCode,
    },
  })
}

// 真实的脚本执行函数 (在 Node.js 后端环境中运行)
async function executeScriptOnRemoteServer(executionId: string, config: ExecuteRequest) {
  console.log(`\n=== Starting Real Script Execution [${executionId}] ===`)
  console.log("Configuration:", JSON.stringify(config, null, 2))

  const execution = executions.get(executionId)
  if (!execution) {
    console.log("❌ Execution not found in executeScriptOnRemoteServer")
    return
  }

  const conn = new Client()
  let timeoutId: NodeJS.Timeout | null = null

  const connectOptions: any = {
    host: config.server,
    port: 22, // Default SSH port
    username: "a",
    readyTimeout: config.timeout * 1000, // Use the configured timeout
  }

  if (config.privateKey) {
    console.log(config.privateKey);
    connectOptions.privateKey = config.privateKey
  } else if (config.password) {
    console.log(config.password);
    connectOptions.password = "a"
  }

  conn
    .on("ready", () => {
      console.log("SSH Client :: ready")
      // 构建命令，包括工作目录和环境变量
      const envVars = Object.entries(config.environmentVars || {})
        .map(([key, value]) => `${key}='${value}'`)
        .join(" ")
      const command = `cd ${config.workingDir || "~"} && ${envVars} ${config.scriptPath} ${config.args || ""}`
      console.log("Executing command:", command)

      // 设置命令执行超时
      timeoutId = setTimeout(() => {
        console.error(`❌ Script execution timed out after ${config.timeout} seconds.`)
        execution.output.push(
          `[${new Date().toISOString()}] ERROR: Script execution timed out after ${config.timeout} seconds.`,
        )
        execution.status = "failed"
        execution.endTime = new Date()
        execution.exitCode = 124 // Common exit code for timeout
        conn.end() // Close connection on timeout
      }, config.timeout * 1000)

      conn.exec(command, { pty: true, env: config.environmentVars }, (err, stream) => {
        if (timeoutId) clearTimeout(timeoutId) // Clear timeout if command starts

        if (err) {
          console.error("SSH Execution Error:", err)
          execution.output.push(`[${new Date().toISOString()}] ERROR: SSH command failed - ${err.message}`)
          execution.status = "failed"
          execution.endTime = new Date()
          execution.exitCode = 1
          conn.end()
          return
        }

        stream
          .on("data", (data: Buffer) => {
            const line = data.toString()
            execution.output.push(`[${new Date().toISOString()}] ${line.trim()}`)
            console.log(`SSH Output: ${line.trim()}`)
          })
          .stderr.on("data", (data: Buffer) => {
            const line = data.toString()
            execution.output.push(`[${new Date().toISOString()}] STDERR: ${line.trim()}`)
            console.error(`SSH STDERR: ${line.trim()}`)
          })
          .on("close", (code: number, signal: string) => {
            console.log("SSH Stream :: close :: code: %d, signal: %s", code, signal)
            execution.status = code === 0 ? "completed" : "failed"
            execution.endTime = new Date()
            execution.exitCode = code
            if (code !== 0) {
              execution.output.push(`[${new Date().toISOString()}] Script exited with code ${code}`)
            }
            conn.end()
            console.log(`=== Real Script Execution Finished [${executionId}] ===\n`)
          })
      })
    })
    .on("error", (err) => {
      if (timeoutId) clearTimeout(timeoutId) // Clear timeout if connection fails
      console.error("SSH Connection Error:", err)
      execution.output.push(`[${new Date().toISOString()}] ERROR: SSH connection failed - ${err.message}`)
      execution.status = "failed"
      execution.endTime = new Date()
      execution.exitCode = 1 // General error code for connection issues
      conn.end()
    })
    .connect(connectOptions)

  // 清理旧的执行记录（5分钟后）
  setTimeout(
    () => {
      console.log(`🗑️ Cleaning up execution record: ${executionId}`)
      executions.delete(executionId)
      console.log(`Remaining executions: ${executions.size}`)
    },
    5 * 60 * 1000,
  )
}
