import { type NextRequest, NextResponse } from "next/server"

interface LogEntry {
  id: string
  timestamp: string
  level: "error" | "warning" | "info" | "debug"
  source: string
  message: string
  details?: string
  type: "system" | "robot" | "script" | "security"
}

// 模拟日志数据
const generateLogs = (type: string, count = 50): LogEntry[] => {
  const levels: LogEntry["level"][] = ["error", "warning", "info", "debug"]
  const sources = {
    system: ["SystemCore", "DatabaseManager", "NetworkManager", "TaskManager"],
    robot: ["R-001", "R-002", "R-003", "R-004", "R-005"],
    script: ["yolox_ros.sh", "gripper.sh", "hitbot.sh", "monitor.sh"],
    security: ["AuthService", "SecurityManager", "AccessControl", "AuditLogger"],
  }

  const messages = {
    error: [
      "Connection timeout occurred",
      "Authentication failed",
      "Database query failed",
      "Service unavailable",
      "Permission denied",
    ],
    warning: [
      "High memory usage detected",
      "Slow response time",
      "Configuration mismatch",
      "Deprecated API usage",
      "Resource limit approaching",
    ],
    info: [
      "Service started successfully",
      "Task completed",
      "Configuration updated",
      "User logged in",
      "Backup completed",
    ],
    debug: ["Processing request", "Cache hit", "Query executed", "Event triggered", "Data synchronized"],
  }

  return Array.from({ length: count }, (_, i) => {
    const level = levels[Math.floor(Math.random() * levels.length)]
    const sourceList = sources[type as keyof typeof sources] || sources.system
    const source = sourceList[Math.floor(Math.random() * sourceList.length)]
    const messageList = messages[level]
    const message = messageList[Math.floor(Math.random() * messageList.length)]

    const timestamp = new Date(Date.now() - Math.random() * 24 * 60 * 60 * 1000)

    return {
      id: `${type}_${i + 1}`,
      timestamp: timestamp.toISOString().replace("T", " ").substring(0, 19),
      level,
      source,
      message,
      details: Math.random() > 0.7 ? `Additional details for ${message.toLowerCase()}` : undefined,
      type: type as LogEntry["type"],
    }
  }).sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime())
}

export async function GET(request: NextRequest) {
  const { searchParams } = new URL(request.url)
  const type = searchParams.get("type") || "system"
  const level = searchParams.get("level")
  const search = searchParams.get("search")
  const limit = Number.parseInt(searchParams.get("limit") || "50")

  let logs = generateLogs(type, limit)

  // 按级别过滤
  if (level && level !== "all") {
    logs = logs.filter((log) => log.level === level)
  }

  // 按搜索词过滤
  if (search) {
    const searchLower = search.toLowerCase()
    logs = logs.filter(
      (log) =>
        log.message.toLowerCase().includes(searchLower) ||
        log.source.toLowerCase().includes(searchLower) ||
        (log.details && log.details.toLowerCase().includes(searchLower)),
    )
  }

  return NextResponse.json({
    success: true,
    logs,
    total: logs.length,
    stats: {
      error: logs.filter((log) => log.level === "error").length,
      warning: logs.filter((log) => log.level === "warning").length,
      info: logs.filter((log) => log.level === "info").length,
      debug: logs.filter((log) => log.level === "debug").length,
    },
  })
}
