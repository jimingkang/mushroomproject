"use client"

import { useState, useEffect, useRef } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import {
  ScrollText,
  Search,
  Trash2,
  Download,
  Pause,
  Play,
  Filter,
} from "lucide-react"

interface LogEntry {
  id: number
  timestamp: string
  level: "info" | "warn" | "error" | "debug"
  module: string
  message: string
}

export function LogViewer() {
  const [logs, setLogs] = useState<LogEntry[]>([])
  const [isPaused, setIsPaused] = useState(false)
  const [filter, setFilter] = useState("")
  const [activeLevel, setActiveLevel] = useState<string | null>(null)

  // 引用 EventSource 以便在组件卸载或暂停时关闭连接
  const eventSourceRef = useRef<EventSource | null>(null)
  const scrollRef = useRef<HTMLDivElement>(null)

  // --- 核心修改：SSE 连接逻辑 ---
  useEffect(() => {
    // 如果暂停了，不仅不更新状态，还要关闭连接以节省资源
    if (isPaused) {
      if (eventSourceRef.current) {
        eventSourceRef.current.close()
        eventSourceRef.current = null
      }
      return
    }

    // 建立 SSE 连接 (指向你的 Flask 地址)
    // 假设 Flask 运行在 172.23.248.32:5002
    const url = "http://172.23.248.32:5002/stream-logs"
    const eventSource = new EventSource(url)
    eventSourceRef.current = eventSource

    // 监听消息事件
    eventSource.onmessage = (event) => {
      try {
        const newLog: LogEntry = JSON.parse(event.data)

        setLogs((prevLogs) => {
          // 保持最近 100 条日志，防止前端内存溢出
          const updatedLogs = [...prevLogs, newLog]
          if (updatedLogs.length > 100) {
            return updatedLogs.slice(updatedLogs.length - 100)
          }
          return updatedLogs
        })
      } catch (error) {
        console.error("Error parsing SSE log data:", error)
      }
    }

    eventSource.onerror = (error) => {
      console.error("SSE Connection Error:", error)
      eventSource.close()
    }

    // 组件卸载时清理连接
    return () => {
      eventSource.close()
    }
  }, [isPaused]) // 依赖 isPaused，切换状态时会重新建立/断开连接

  // 自动滚动到底部
  useEffect(() => {
    if (!isPaused && scrollRef.current) {
      // 使用 requestAnimationFrame 确保在 DOM 渲染后滚动
      requestAnimationFrame(() => {
        if (scrollRef.current) {
          scrollRef.current.scrollTop = scrollRef.current.scrollHeight
        }
      })
    }
  }, [logs, isPaused])

  // --- 过滤逻辑 (保持不变) ---
  const filteredLogs = logs.filter((log) => {
    const matchesFilter =
        filter === "" ||
        log.message.toLowerCase().includes(filter.toLowerCase()) ||
        log.module.toLowerCase().includes(filter.toLowerCase())
    const matchesLevel = activeLevel === null || log.level === activeLevel
    return matchesFilter && matchesLevel
  })

  // --- 样式辅助函数 (保持不变) ---
  const getLevelColor = (level: string) => {
    switch (level) {
      case "error": return "text-destructive"
      case "warn": return "text-warning"
      case "info": return "text-info"
      case "debug": return "text-muted-foreground"
      default: return "text-foreground"
    }
  }

  const getLevelBadge = (level: string) => {
    switch (level) {
      case "error": return "bg-destructive/20 text-destructive border-destructive/30"
      case "warn": return "bg-warning/20 text-warning border-warning/30"
      case "info": return "bg-info/20 text-info border-info/30"
      case "debug": return "bg-muted text-muted-foreground border-muted"
      default: return ""
    }
  }

  return (
      <Card className="border-border bg-card h-full flex flex-col">
        <CardHeader className="flex flex-row items-center justify-between py-3 px-4 border-b border-border">
          <div className="flex items-center gap-3">
            <ScrollText className="h-4 w-4 text-muted-foreground" />
            <CardTitle className="text-sm font-medium">System Logs (SSE)</CardTitle>
            <Badge variant="secondary" className="text-xs font-mono">
              {isPaused ? "Paused" : "Live Streaming"}
            </Badge>
          </div>
          <div className="flex items-center gap-2">
            <Button
                variant="ghost"
                size="icon"
                className="h-7 w-7"
                onClick={() => setIsPaused(!isPaused)}
            >
              {isPaused ? <Play className="h-4 w-4" /> : <Pause className="h-4 w-4" />}
            </Button>
            <Button
                variant="ghost"
                size="icon"
                className="h-7 w-7"
                onClick={() => setLogs([])}
            >
              <Trash2 className="h-4 w-4" />
            </Button>
          </div>
        </CardHeader>

        {/* 过滤器区域 */}
        <div className="px-4 py-2 border-b border-border flex flex-wrap items-center gap-2">
          <div className="relative flex-1 min-w-[200px]">
            <Search className="absolute left-2 top-1/2 -translate-y-1/2 h-4 w-4 text-muted-foreground" />
            <Input
                placeholder="Filter logs..."
                value={filter}
                onChange={(e) => setFilter(e.target.value)}
                className="pl-8 h-8 text-sm bg-secondary border-border"
            />
          </div>
          <div className="flex items-center gap-1">
            <Filter className="h-4 w-4 text-muted-foreground mr-1" />
            {["error", "warn", "info", "debug"].map((level) => (
                <Button
                    key={level}
                    variant={activeLevel === level ? "default" : "ghost"}
                    size="sm"
                    className={`h-7 text-xs ${activeLevel === level ? "" : getLevelColor(level)}`}
                    onClick={() => setActiveLevel(activeLevel === level ? null : level)}
                >
                  {level.toUpperCase()}
                </Button>
            ))}
          </div>
        </div>

        <CardContent className="p-0 flex-1 overflow-hidden">
          <div ref={scrollRef} className="h-full overflow-y-auto font-mono text-xs">
            {logs.length === 0 && !isPaused && (
                <div className="p-4 text-center text-muted-foreground">
                  Connecting to log stream...
                </div>
            )}
            {filteredLogs.map((log) => (
                <div
                    key={log.id}
                    className="flex items-start gap-3 px-4 py-1.5 hover:bg-secondary/50 border-b border-border/50 animate-in fade-in duration-300"
                >
              <span className="text-muted-foreground w-24 shrink-0">
                {log.timestamp}
              </span>
                  <Badge
                      variant="outline"
                      className={`${getLevelBadge(log.level)} w-14 justify-center text-[10px] shrink-0`}
                  >
                    {log.level.toUpperCase()}
                  </Badge>
                  <span className="text-primary w-16 shrink-0">[{log.module}]</span>
                  <span className={`${getLevelColor(log.level)} flex-1 break-all`}>
                {log.message}
              </span>
                </div>
            ))}
          </div>
        </CardContent>
      </Card>
  )
}