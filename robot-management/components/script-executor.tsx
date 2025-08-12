"use client"

import { useState, useEffect } from "react"
import { Play, Square, Terminal, Server, Settings } from "lucide-react"

import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Textarea } from "@/components/ui/textarea"
import { ScrollArea } from "@/components/ui/scroll-area"
import { Badge } from "@/components/ui/badge"
import { useToast } from "@/hooks/use-toast"

interface ExecutionConfig {
  server: string
  scriptPath: string
  args: string
  workingDir: string
  environmentVars: string
  timeout: number
  username: string // 添加用户名
  password?: string // 添加密码
  privateKey?: string // 添加私钥
}

export function ScriptExecutor() {
  const [isExecuting, setIsExecuting] = useState(false)
  const [output, setOutput] = useState("")
  const [executionId, setExecutionId] = useState<string | null>(null)
  const [config, setConfig] = useState<ExecutionConfig>({
    server: "172.27.34.62",
    scriptPath: "/home/a/Downloads/mushroomproject/ros2_ws/camera.sh",
    args: "",
    workingDir: "/home/a/Downloads/mushroomproject/ros2_ws",
    environmentVars: "",
    timeout: 300,
    username: "a", // 默认用户名
    password: "a", // 默认密码
    privateKey: "", // 默认私钥
  })

  const { toast } = useToast()

  // 轮询执行状态
  useEffect(() => {
    if (!executionId || !isExecuting) {
      if (executionId && !isExecuting) {
        console.log("Polling stopped - execution completed")
      }
      return
    }

    console.log(`Starting polling for execution: ${executionId}`)

    const pollInterval = setInterval(async () => {
      try {
        console.log(`Polling status for execution: ${executionId}`)
        const response = await fetch(`/api/scripts/execute?executionId=${executionId}`)

        if (!response.ok) {
          console.log(`Polling response not OK: ${response.status}`)
          return
        }

        const data = await response.json()
        console.log("Polling response:", {
          success: data.success,
          status: data.execution?.status,
          outputLines: data.execution?.output?.length || 0,
          exitCode: data.execution?.exitCode,
        })

        if (data.success && data.execution) {
          const { status, output: executionOutput, exitCode } = data.execution

          // 更新输出
          const newOutput = executionOutput.join("\n")
          if (newOutput !== output) {
            console.log(`Output updated: ${executionOutput.length} lines`)
            setOutput(newOutput)
          }

          // 检查是否完成
          if (status === "completed" || status === "failed") {
            console.log(`✅ Execution ${status}:`, {
              executionId,
              exitCode,
              duration: data.execution.endTime
                ? new Date(data.execution.endTime).getTime() - new Date(data.execution.startTime).getTime()
                : "unknown",
            })

            setIsExecuting(false)
            clearInterval(pollInterval)

            toast({
              title: status === "completed" ? "Script Completed" : "Script Failed",
              description: `Exit code: ${exitCode}`,
              variant: status === "completed" ? "default" : "destructive",
            })
          }
        }
      } catch (error) {
        console.error("❌ Polling error:", error)
        console.error("Polling error details:", {
          executionId,
          error: error instanceof Error ? error.message : String(error),
        })
      }
    }, 1000)

    return () => {
      console.log(`Cleaning up polling interval for: ${executionId}`)
      clearInterval(pollInterval)
    }
  }, [executionId, isExecuting, toast, output])

  const handleExecute = async () => {
    console.log("=== Frontend Script Execution Started ===")
    console.log("Timestamp:", new Date().toISOString())
    console.log("Configuration:", config)

    try {
      setIsExecuting(true)
      setOutput("Initializing script execution...\n")

      // 解析环境变量
      const environmentVars: Record<string, string> = {}
      if (config.environmentVars) {
        console.log("Parsing environment variables...")
        config.environmentVars.split("\n").forEach((line, index) => {
          const [key, value] = line.split("=")
          if (key && value) {
            environmentVars[key.trim()] = value.trim()
            console.log(`  ${index + 1}. ${key.trim()} = ${value.trim()}`)
          }
        })
        console.log(`Parsed ${Object.keys(environmentVars).length} environment variables`)
      }

      const requestPayload = {
        server: config.server,
        scriptPath: config.scriptPath,
        args: config.args,
        workingDir: config.workingDir,
        environmentVars,
        timeout: config.timeout,
        username: config.username,
        password: config.password,
        privateKey: config.privateKey,
      }

      console.log("Sending request to /api/scripts/execute")
      console.log("Request payload:", JSON.stringify(requestPayload, null, 2))

      const response = await fetch("/api/scripts/execute", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(requestPayload),
      })

      console.log("Response status:", response.status)
      console.log("Response headers:", Object.fromEntries(response.headers.entries()))

      const data = await response.json()
      console.log("Response data:", data)

      if (data.success) {
        setExecutionId(data.executionId)
        console.log("✅ Script execution started successfully")
        console.log("Execution ID:", data.executionId)

        toast({
          title: "Script Started",
          description: "Script execution has been initiated",
        })
      } else {
        console.log("❌ Script execution failed:", data.error)
        setIsExecuting(false)
        setOutput((prev) => prev + `Error: ${data.error}\n`)
        toast({
          title: "Execution Failed",
          description: data.error,
          variant: "destructive",
        })
      }
    } catch (error) {
      console.error("❌ Frontend execution error:", error)
      console.error("Error details:", {
        name: error instanceof Error ? error.name : "Unknown",
        message: error instanceof Error ? error.message : String(error),
        stack: error instanceof Error ? error.stack : "No stack trace",
      })

      setIsExecuting(false)
      setOutput((prev) => prev + `Error: ${error}\n`)
      toast({
        title: "Connection Error",
        description: "Failed to connect to server",
        variant: "destructive",
      })
    }

    console.log("=== Frontend Script Execution Request Completed ===\n")
  }

  const handleStop = () => {
    console.log("=== Script Execution Stop Requested ===")
    console.log("Execution ID:", executionId)
    console.log("Was executing:", isExecuting)

    setIsExecuting(false)
    setExecutionId(null)
    setOutput((prev) => prev + "\n[STOPPED] Script execution stopped by user\n")

    console.log("✅ Script execution stopped by user")

    toast({
      title: "Script Stopped",
      description: "Script execution has been terminated",
    })
  }

  const handleConfigChange = (field: keyof ExecutionConfig, value: string | number) => {
    setConfig((prev) => ({ ...prev, [field]: value }))
  }

  return (
    <div className="grid gap-6 lg:grid-cols-2">
      <Card>
        <CardHeader>
          <CardTitle>Script Configuration</CardTitle>
          <CardDescription>Configure and execute Linux scripts remotely</CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="space-y-2">
            <Label htmlFor="server">Target Server</Label>
            <Select value={config.server} onValueChange={(value) => handleConfigChange("server", value)}>
              <SelectTrigger id="server">
                <SelectValue placeholder="Select server" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="172.27.34.62">jeson orin nano (172.27.34.62)</SelectItem>
                <SelectItem value="web-server-01">rpi5 (172.27.34.66)</SelectItem>
                <SelectItem value="app-server-01">app-server-01 (192.168.1.12)</SelectItem>
                <SelectItem value="monitor-server-01">monitor-server-01 (192.168.1.13)</SelectItem>
              </SelectContent>
            </Select>
          </div>

          <div className="space-y-2">
            <Label htmlFor="script-path">Script Path</Label>
            <Input
              id="script-path"
              value={config.scriptPath}
              onChange={(e) => handleConfigChange("scriptPath", e.target.value)}
              placeholder="/opt/scripts/backup_database.sh"
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="working-dir">Working Directory</Label>
            <Input
              id="working-dir"
              value={config.workingDir}
              onChange={(e) => handleConfigChange("workingDir", e.target.value)}
              placeholder="/opt/scripts"
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="arguments">Arguments</Label>
            <Input
              id="arguments"
              value={config.args}
              onChange={(e) => handleConfigChange("args", e.target.value)}
              placeholder="--verbose --backup-dir=/backups"
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="environment">Environment Variables</Label>
            <Textarea
              id="environment"
              value={config.environmentVars}
              onChange={(e) => handleConfigChange("environmentVars", e.target.value)}
              placeholder="KEY1=value1&#10;KEY2=value2"
              rows={3}
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="timeout">Timeout (seconds)</Label>
            <Input
              id="timeout"
              type="number"
              value={config.timeout}
              onChange={(e) => handleConfigChange("timeout", Number.parseInt(e.target.value) || 300)}
              placeholder="300"
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="username">SSH Username</Label>
            <Input
              id="username"
              value={config.username}
              onChange={(e) => handleConfigChange("username", "a")}
              placeholder="e.g., root or ubuntu"
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="password">SSH Password (Optional)</Label>
            <Input
              id="password"
              type="password"
              value={config.password}
              onChange={(e) => handleConfigChange("password", e.target.value)}
              placeholder="a"
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="private-key">SSH Private Key (Optional)</Label>
            <Textarea
              id="private-key"
              value={config.privateKey}
              onChange={(e) => handleConfigChange("privateKey", e.target.value)}
              placeholder=""
              rows={5}
            />
          </div>

          <div className="flex gap-2 pt-4">
            <Button onClick={handleExecute} disabled={isExecuting} className="flex-1">
              <Play className="mr-2 h-4 w-4" />
              {isExecuting ? "Executing..." : "Execute Script"}
            </Button>
            {isExecuting && (
              <Button variant="outline" onClick={handleStop}>
                <Square className="mr-2 h-4 w-4" />
                Stop
              </Button>
            )}
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle>Execution Output</CardTitle>
              <CardDescription>Real-time script execution results</CardDescription>
            </div>
            <div className="flex items-center gap-2">
              {isExecuting && (
                <Badge variant="default" className="bg-green-500">
                  <Terminal className="mr-1 h-3 w-3" />
                  Running
                </Badge>
              )}
              {executionId && (
                <Badge variant="outline" className="text-xs">
                  ID: {executionId.split("_")[2]}
                </Badge>
              )}
            </div>
          </div>
        </CardHeader>
        <CardContent>
          <ScrollArea className="h-[400px] w-full rounded-md border bg-black p-4">
            <pre className="text-sm text-green-400 font-mono whitespace-pre-wrap">
              {output || "No output yet. Execute a script to see results here."}
            </pre>
          </ScrollArea>
          <div className="flex justify-between items-center mt-4 text-sm text-muted-foreground">
            <div className="flex items-center gap-2">
              <Server className="h-4 w-4" />
              <span>Connected to: {config.server}</span>
            </div>
            <div className="flex items-center gap-2">
              <Settings className="h-4 w-4" />
              <span>Timeout: {config.timeout}s</span>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
