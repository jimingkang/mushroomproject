"use client"

import { Play, Square, Edit, Trash2, FileText, Clock, Server, MoreHorizontal } from "lucide-react"
import { useState, useEffect } from "react"

import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu"
import { useToast } from "@/hooks/use-toast"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Textarea } from "@/components/ui/textarea"

interface Script {
  id: string
  name: string
  description: string
  path: string
  status: "idle" | "running" | "scheduled"
  lastRun: string
  server: string
  category: string
}

export function ScriptList() {
  const [scripts, setScripts] = useState<Script[]>([])
  const [loading, setLoading] = useState(true)
  const { toast } = useToast()
  const [sshUsername, setSshUsername] = useState("a") // Default username
  const [sshPassword, setSshPassword] = useState("a")
  const [sshPrivateKey, setSshPrivateKey] = useState("")

  useEffect(() => {
    fetchScripts()
  }, [])

  const fetchScripts = async () => {
    console.log("=== Fetching Scripts List ===")
    console.log("Timestamp:", new Date().toISOString())

    try {
      const response = await fetch("/api/scripts/list")
      console.log("Scripts list response status:", response.status)

      const data = await response.json()
      console.log("Scripts list response data:", {
        success: data.success,
        scriptsCount: data.scripts?.length || 0,
        total: data.total,
      })

      if (data.success) {
        setScripts(data.scripts)
        console.log("✅ Scripts loaded successfully:", data.scripts.length, "scripts")
        data.scripts.forEach((script: Script, index: number) => {
          console.log(`  ${index + 1}. ${script.name} (${script.status}) - ${script.server}`)
        })
      }
    } catch (error) {
      console.error("❌ Failed to fetch scripts:", error)
      toast({
        title: "Error",
        description: "Failed to load scripts",
        variant: "destructive",
      })
    } finally {
      setLoading(false)
      console.log("Scripts loading completed")
    }
  }

  const handleExecuteScript = async (script: Script) => {
    console.log("=== Quick Script Execution from List ===")
    console.log("Script details:", {
      id: script.id,
      name: script.name,
      path: script.path,
      server: script.server,
      currentStatus: script.status,
    })

    try {
      const requestPayload = {
        server: script.server,
        scriptPath: script.path,
        workingDir: "/opt/scripts",
        username: sshUsername, // Add this line
        password: sshPassword, // Add this line
        privateKey: sshPrivateKey, // Add this line
      }

      console.log("Sending quick execution request:", requestPayload)

      const response = await fetch("/api/scripts/execute", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(requestPayload),
      })

      console.log("Quick execution response status:", response.status)
      const data = await response.json()
      console.log("Quick execution response data:", data)

      if (data.success) {
        console.log("✅ Quick script execution started:", data.executionId)

        toast({
          title: "Script Started",
          description: `${script.name} execution initiated`,
        })

        // 更新脚本状态
        setScripts((prev) => prev.map((s) => (s.id === script.id ? { ...s, status: "running" as const } : s)))
        console.log(`Updated script ${script.id} status to running`)
      } else {
        console.log("❌ Quick script execution failed:", data.error)
        toast({
          title: "Execution Failed",
          description: data.error,
          variant: "destructive",
        })
      }
    } catch (error) {
      console.error("❌ Quick execution error:", error)
      toast({
        title: "Error",
        description: "Failed to execute script",
        variant: "destructive",
      })
    }
  }

  const handleDeleteScript = async (scriptId: string) => {
    try {
      const response = await fetch(`/api/scripts/${scriptId}`, {
        method: "DELETE",
      })

      const data = await response.json()

      if (data.success) {
        setScripts((prev) => prev.filter((s) => s.id !== scriptId))
        toast({
          title: "Script Deleted",
          description: "Script has been removed successfully",
        })
      } else {
        toast({
          title: "Delete Failed",
          description: data.error,
          variant: "destructive",
        })
      }
    } catch (error) {
      toast({
        title: "Error",
        description: "Failed to delete script",
        variant: "destructive",
      })
    }
  }

  const getStatusBadge = (status: string) => {
    switch (status) {
      case "running":
        return (
          <Badge variant="default" className="bg-green-500">
            Running
          </Badge>
        )
      case "scheduled":
        return (
          <Badge variant="outline" className="border-blue-500 text-blue-500">
            Scheduled
          </Badge>
        )
      default:
        return <Badge variant="secondary">Idle</Badge>
    }
  }

  const getCategoryColor = (category: string) => {
    const colors = {
      Backup: "bg-blue-100 text-blue-800",
      Maintenance: "bg-yellow-100 text-yellow-800",
      Deployment: "bg-green-100 text-green-800",
      Monitoring: "bg-purple-100 text-purple-800",
    }
    return colors[category as keyof typeof colors] || "bg-gray-100 text-gray-800"
  }

  if (loading) {
    return <div className="text-center py-8">Loading scripts...</div>
  }

  return (
    <Card className="mb-6">
      <CardHeader>
        <CardTitle>Global SSH Configuration for Quick Execute</CardTitle>
        <CardDescription>
          Enter SSH credentials to be used for quick execution of scripts from the list.
          <span className="text-destructive-foreground">
            For production, consider more secure ways to manage credentials (e.g., environment variables, secure vault).
          </span>
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div className="space-y-2">
            <Label htmlFor="ssh-username">SSH Username</Label>
            <Input
              id="ssh-username"
              value={sshUsername}
              onChange={(e) => setSshUsername("a")}
              placeholder="e.g., root or ubuntu"
            />
          </div>
          <div className="space-y-2">
            <Label htmlFor="ssh-password">SSH Password (Optional)</Label>
            <Input
              id="ssh-password"
              type="password"
              value={sshPassword}
              onChange={(e) => setSshPassword("a")}
              placeholder="Leave empty if using private key"
            />
          </div>
        </div>
        <div className="space-y-2">
          <Label htmlFor="ssh-private-key">SSH Private Key (Optional)</Label>
          <Textarea
            id="ssh-private-key"
            value={sshPrivateKey}
            onChange={(e) => setSshPrivateKey(e.target.value)}
            placeholder="Paste your private key here (e.g., from ~/.ssh/id_rsa)"
            rows={5}
          />
        </div>
      </CardContent>
    </Card>
  )

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
      {scripts.map((script) => (
        <Card key={script.id}>
          <CardHeader className="pb-2">
            <div className="flex items-start justify-between">
              <div className="flex-1">
                <CardTitle className="text-base">{script.name}</CardTitle>
                <CardDescription className="mt-1">{script.description}</CardDescription>
              </div>
              <DropdownMenu>
                <DropdownMenuTrigger asChild>
                  <Button variant="ghost" size="icon">
                    <MoreHorizontal className="h-4 w-4" />
                    <span className="sr-only">Open menu</span>
                  </Button>
                </DropdownMenuTrigger>
                <DropdownMenuContent align="end">
                  <DropdownMenuLabel>Actions</DropdownMenuLabel>
                  <DropdownMenuItem onClick={() => handleExecuteScript(script)}>
                    <Play className="mr-2 h-4 w-4" />
                    Execute
                  </DropdownMenuItem>
                  <DropdownMenuItem>
                    <Edit className="mr-2 h-4 w-4" />
                    Edit
                  </DropdownMenuItem>
                  <DropdownMenuItem>
                    <FileText className="mr-2 h-4 w-4" />
                    View Logs
                  </DropdownMenuItem>
                  <DropdownMenuSeparator />
                  <DropdownMenuItem className="text-destructive" onClick={() => handleDeleteScript(script.id)}>
                    <Trash2 className="mr-2 h-4 w-4" />
                    Delete
                  </DropdownMenuItem>
                </DropdownMenuContent>
              </DropdownMenu>
            </div>
            <div className="flex items-center gap-2 pt-2">
              {getStatusBadge(script.status)}
              <Badge variant="secondary" className={getCategoryColor(script.category)}>
                {script.category}
              </Badge>
            </div>
          </CardHeader>
          <CardContent className="pb-2">
            <div className="space-y-2 text-sm">
              <div className="flex items-center text-muted-foreground">
                <FileText className="mr-2 h-4 w-4" />
                <span className="truncate">{script.path}</span>
              </div>
              <div className="flex items-center text-muted-foreground">
                <Server className="mr-2 h-4 w-4" />
                <span>{script.server}</span>
              </div>
              <div className="flex items-center text-muted-foreground">
                <Clock className="mr-2 h-4 w-4" />
                <span>Last run: {script.lastRun}</span>
              </div>
            </div>
          </CardContent>
          <div className="flex gap-2 p-4 pt-0">
            <Button
              size="sm"
              className="flex-1"
              disabled={script.status === "running"}
              onClick={() => handleExecuteScript(script)}
            >
              <Play className="mr-2 h-4 w-4" />
              {script.status === "running" ? "Running..." : "Execute"}
            </Button>
            {script.status === "running" && (
              <Button variant="outline" size="sm">
                <Square className="mr-2 h-4 w-4" />
                Stop
              </Button>
            )}
          </div>
        </Card>
      ))}
    </div>
  )
}
