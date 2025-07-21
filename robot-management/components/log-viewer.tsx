import { AlertTriangle, CheckCircle, Info, XCircle, Clock, Server } from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Card, CardContent } from "@/components/ui/card"
import { ScrollArea } from "@/components/ui/scroll-area"

interface LogViewerProps {
  logType: "system" | "robot" | "script" | "security"
}

interface LogEntry {
  id: string
  timestamp: string
  level: "error" | "warning" | "info" | "debug"
  source: string
  message: string
  details?: string
}

export function LogViewer({ logType }: LogViewerProps) {
  // Mock data - in a real app, this would come from an API
  const logs: LogEntry[] = [
    {
      id: "1",
      timestamp: "2025-01-17 14:22:15",
      level: "error",
      source: logType === "robot" ? "R-001" : logType === "script" ? "backup_script.sh" : "SystemCore",
      message:
        logType === "robot"
          ? "Battery level critically low"
          : logType === "script"
            ? "Script execution failed: Permission denied"
            : "Database connection timeout",
      details:
        logType === "robot"
          ? "Battery at 5%, immediate charging required"
          : logType === "script"
            ? "Failed to execute /opt/scripts/backup_script.sh - insufficient permissions"
            : "Connection to primary database failed after 30 seconds",
    },
    {
      id: "2",
      timestamp: "2025-01-17 14:20:33",
      level: "warning",
      source: logType === "robot" ? "R-003" : logType === "script" ? "maintenance.sh" : "AuthService",
      message:
        logType === "robot"
          ? "Sensor calibration drift detected"
          : logType === "script"
            ? "Script completed with warnings"
            : "Multiple failed login attempts detected",
      details:
        logType === "robot"
          ? "Proximity sensor showing 2.3% drift from baseline"
          : logType === "script"
            ? "Some files could not be processed during maintenance routine"
            : "5 failed attempts from IP 192.168.1.100",
    },
    {
      id: "3",
      timestamp: "2025-01-17 14:18:45",
      level: "info",
      source: logType === "robot" ? "R-002" : logType === "script" ? "deploy.sh" : "TaskManager",
      message:
        logType === "robot"
          ? "Task completed successfully"
          : logType === "script"
            ? "Deployment script executed successfully"
            : "Scheduled maintenance completed",
      details:
        logType === "robot"
          ? "Assembly task T-1002 completed in 1h 45m"
          : logType === "script"
            ? "Application deployed to production environment"
            : "All robots updated with latest firmware",
    },
    {
      id: "4",
      timestamp: "2025-01-17 14:15:22",
      level: "info",
      source: logType === "robot" ? "R-004" : logType === "script" ? "monitor.sh" : "NetworkManager",
      message:
        logType === "robot"
          ? "Robot status updated"
          : logType === "script"
            ? "System monitoring started"
            : "Network connectivity restored",
      details:
        logType === "robot"
          ? "Status changed from idle to active"
          : logType === "script"
            ? "Monitoring services initialized successfully"
            : "Connection to external services re-established",
    },
    {
      id: "5",
      timestamp: "2025-01-17 14:12:10",
      level: "debug",
      source: logType === "robot" ? "R-001" : logType === "script" ? "cleanup.sh" : "ConfigManager",
      message:
        logType === "robot"
          ? "Sensor data received"
          : logType === "script"
            ? "Temporary files cleaned"
            : "Configuration reloaded",
      details:
        logType === "robot"
          ? "Temperature: 42°C, Humidity: 65%, Pressure: 1013hPa"
          : logType === "script"
            ? "Removed 1.2GB of temporary files"
            : "System configuration updated from config.yaml",
    },
  ]

  const getLevelIcon = (level: string) => {
    switch (level) {
      case "error":
        return <XCircle className="h-4 w-4 text-destructive" />
      case "warning":
        return <AlertTriangle className="h-4 w-4 text-yellow-500" />
      case "info":
        return <Info className="h-4 w-4 text-blue-500" />
      case "debug":
        return <CheckCircle className="h-4 w-4 text-green-500" />
      default:
        return <Info className="h-4 w-4" />
    }
  }

  const getLevelBadge = (level: string) => {
    const variants = {
      error: "destructive" as const,
      warning: "outline" as const,
      info: "default" as const,
      debug: "secondary" as const,
    }
    return variants[level as keyof typeof variants] || "default"
  }

  return (
    <Card>
      <CardContent className="p-0">
        <ScrollArea className="h-[600px]">
          <div className="space-y-2 p-4">
            {logs.map((log) => (
              <div
                key={log.id}
                className="flex items-start gap-3 rounded-lg border p-3 hover:bg-muted/50 transition-colors"
              >
                <div className="flex-shrink-0 mt-0.5">{getLevelIcon(log.level)}</div>
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2 mb-1">
                    <Badge variant={getLevelBadge(log.level)} className="text-xs">
                      {log.level.toUpperCase()}
                    </Badge>
                    <div className="flex items-center text-xs text-muted-foreground">
                      <Clock className="mr-1 h-3 w-3" />
                      {log.timestamp}
                    </div>
                    <div className="flex items-center text-xs text-muted-foreground">
                      <Server className="mr-1 h-3 w-3" />
                      {log.source}
                    </div>
                  </div>
                  <p className="text-sm font-medium mb-1">{log.message}</p>
                  {log.details && <p className="text-xs text-muted-foreground">{log.details}</p>}
                </div>
              </div>
            ))}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  )
}
