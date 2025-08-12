"use client" // Make this a client component

import type React from "react"

import Link from "next/link"
import { ArrowLeft, BatteryCharging, Settings } from "lucide-react"
import { useState } from "react" // Import useState
import { useRouter } from "next/navigation" // Import useRouter
import { useToast } from "@/hooks/use-toast" // Import useToast

import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Textarea } from "@/components/ui/textarea"
import { Switch } from "@/components/ui/switch"

export default function NewScriptPage() {
  const router = useRouter()
  const { toast } = useToast()

  // State for form fields
  const [scriptName, setScriptName] = useState("")
  const [category, setCategory] = useState("")
  const [description, setDescription] = useState("")
  const [scriptPath, setScriptPath] = useState("")
  const [targetServer, setTargetServer] = useState("")
  const [workingDirectory, setWorkingDirectory] = useState("")
  const [timeout, setTimeoutValue] = useState(300)
  const [defaultArgs, setDefaultArgs] = useState("")
  const [environmentVars, setEnvironmentVars] = useState("")
  const [autoRetry, setAutoRetry] = useState(false)
  const [logOutput, setLogOutput] = useState(true)
  const [emailNotifications, setEmailNotifications] = useState(false)
  const [isSubmitting, setIsSubmitting] = useState(false) // Loading state

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault() // Prevent default form submission
    setIsSubmitting(true)

    // Basic validation
    if (!scriptName || !scriptPath || !targetServer) {
      toast({
        title: "Validation Error",
        description: "Script Name, Script Path, and Target Server are required.",
        variant: "destructive",
      })
      setIsSubmitting(false)
      return
    }

    const scriptData = {
      name: scriptName,
      category: category || "Other", // Default category if not selected
      description: description,
      path: scriptPath,
      server: targetServer,
      // Note: The backend API (app/api/scripts/list/route.ts) currently
      // only processes the above fields. Additional fields like workingDirectory,
      // timeout, defaultArgs, environmentVars, autoRetry, logOutput, emailNotifications
      // would require backend modifications to be stored.
      // For now, we only send what the backend expects.
    }

    console.log("Submitting script data:", scriptData)

    try {
      const response = await fetch("/api/scripts/list", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(scriptData),
      })

      const data = await response.json()

      if (data.success) {
        toast({
          title: "Script Added",
          description: `${scriptName} has been registered successfully.`,
        })
        router.refresh() // Add this line to re-fetch data on the /scripts page
        router.push("/scripts") // Redirect to scripts list page
      } else {
        toast({
          title: "Failed to Add Script",
          description: data.error || "An unknown error occurred.",
          variant: "destructive",
        })
      }
    } catch (error) {
      console.error("Error adding script:", error)
      toast({
        title: "Error",
        description: "Could not connect to the server to add script.",
        variant: "destructive",
      })
    } finally {
      setIsSubmitting(false)
    }
  }

  return (
    <div className="flex min-h-screen w-full flex-col">
      <header className="sticky top-0 z-10 flex h-16 items-center gap-4 border-b bg-background px-4 md:px-6">
        <Link href="/" className="flex items-center gap-2 font-semibold">
          <BatteryCharging className="h-6 w-6" />
          <span>RoboManager</span>
        </Link>
        <nav className="ml-auto flex gap-2">
          <Button variant="outline" size="sm" asChild>
            <Link href="/settings">
              <Settings className="mr-2 h-4 w-4" />
              Settings
            </Link>
          </Button>
        </nav>
      </header>
      <div className="flex items-center gap-4 border-b bg-background px-4 py-4 md:px-6">
        <Button variant="ghost" size="icon" asChild>
          <Link href="/scripts">
            <ArrowLeft className="h-4 w-4" />
            <span className="sr-only">Back</span>
          </Link>
        </Button>
        <div>
          <h1 className="text-xl font-bold tracking-tight">Add New Script</h1>
          <p className="text-muted-foreground">Register a new Linux script for remote execution</p>
        </div>
      </div>
      <main className="flex flex-1 flex-col gap-4 p-4 md:gap-8 md:p-8">
        <Card className="mx-auto max-w-2xl">
          <CardHeader>
            <CardTitle>Script Information</CardTitle>
            <CardDescription>Enter the details of the new script</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <form onSubmit={handleSubmit} className="space-y-4">
              {" "}
              {/* Wrap content in form */}
              <div className="grid grid-cols-2 gap-4">
                <div className="space-y-2">
                  <Label htmlFor="script-name">Script Name</Label>
                  <Input
                    id="script-name"
                    placeholder="e.g., backup_database.sh"
                    value={scriptName}
                    onChange={(e) => setScriptName(e.target.value)}
                    required
                  />
                </div>
                <div className="space-y-2">
                  <Label htmlFor="category">Category</Label>
                  <Select value={category} onValueChange={setCategory}>
                    <SelectTrigger id="category">
                      <SelectValue placeholder="Select category" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="backup">Backup</SelectItem>
                      <SelectItem value="maintenance">Maintenance</SelectItem>
                      <SelectItem value="deployment">Deployment</SelectItem>
                      <SelectItem value="monitoring">Monitoring</SelectItem>
                      <SelectItem value="security">Security</SelectItem>
                      <SelectItem value="other">Other</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </div>
              <div className="space-y-2">
                <Label htmlFor="description">Description</Label>
                <Textarea
                  id="description"
                  placeholder="Brief description of what this script does..."
                  rows={2}
                  value={description}
                  onChange={(e) => setDescription(e.target.value)}
                />
              </div>
              <div className="grid grid-cols-2 gap-4">
                <div className="space-y-2">
                  <Label htmlFor="script-path">Script Path</Label>
                  <Input
                    id="script-path"
                    placeholder="/opt/scripts/backup_database.sh"
                    value={scriptPath}
                    onChange={(e) => setScriptPath(e.target.value)}
                    required
                  />
                </div>
                <div className="space-y-2">
                  <Label htmlFor="target-server">Target Server</Label>
                  <Select value={targetServer} onValueChange={setTargetServer} required>
                    <SelectTrigger id="target-server">
                      <SelectValue placeholder="Select server" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="db-server-01">db-server-01</SelectItem>
                      <SelectItem value="web-server-01">web-server-01</SelectItem>
                      <SelectItem value="app-server-01">app-server-01</SelectItem>
                      <SelectItem value="monitor-server-01">monitor-server-01</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </div>
              <div className="grid grid-cols-2 gap-4">
                <div className="space-y-2">
                  <Label htmlFor="working-directory">Working Directory</Label>
                  <Input
                    id="working-directory"
                    placeholder="/opt/scripts"
                    value={workingDirectory}
                    onChange={(e) => setWorkingDirectory(e.target.value)}
                  />
                </div>
                <div className="space-y-2">
                  <Label htmlFor="timeout">Timeout (seconds)</Label>
                  <Input
                    id="timeout"
                    type="number"
                    placeholder="300"
                    value={timeout}
                    onChange={(e) => setTimeoutValue(Number.parseInt(e.target.value) || 300)}
                  />
                </div>
              </div>
              <div className="space-y-2">
                <Label htmlFor="default-args">Default Arguments</Label>
                <Input
                  id="default-args"
                  placeholder="--verbose --log-level=info"
                  value={defaultArgs}
                  onChange={(e) => setDefaultArgs(e.target.value)}
                />
              </div>
              <div className="space-y-2">
                <Label htmlFor="environment-vars">Environment Variables</Label>
                <Textarea
                  id="environment-vars"
                  placeholder="KEY1=value1&#10;KEY2=value2"
                  rows={3}
                  value={environmentVars}
                  onChange={(e) => setEnvironmentVars(e.target.value)}
                />
              </div>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <div className="space-y-0.5">
                    <Label htmlFor="auto-retry">Auto Retry on Failure</Label>
                    <p className="text-sm text-muted-foreground">Automatically retry script execution if it fails</p>
                  </div>
                  <Switch id="auto-retry" checked={autoRetry} onCheckedChange={setAutoRetry} />
                </div>

                <div className="flex items-center justify-between">
                  <div className="space-y-0.5">
                    <Label htmlFor="log-output">Log Output</Label>
                    <p className="text-sm text-muted-foreground">Save script output to system logs</p>
                  </div>
                  <Switch id="log-output" checked={logOutput} onCheckedChange={setLogOutput} />
                </div>

                <div className="flex items-center justify-between">
                  <div className="space-y-0.5">
                    <Label htmlFor="email-notifications">Email Notifications</Label>
                    <p className="text-sm text-muted-foreground">Send email notifications on completion or failure</p>
                  </div>
                  <Switch
                    id="email-notifications"
                    checked={emailNotifications}
                    onCheckedChange={setEmailNotifications}
                  />
                </div>
              </div>
              <CardFooter className="flex justify-between p-0 pt-4">
                {" "}
                {/* Adjust padding as form is inside */}
                <Button variant="outline" asChild>
                  <Link href="/scripts">Cancel</Link>
                </Button>
                <Button type="submit" disabled={isSubmitting}>
                  {isSubmitting ? "Adding Script..." : "Add Script"}
                </Button>
              </CardFooter>
            </form>
          </CardContent>
        </Card>
      </main>
    </div>
  )
}
