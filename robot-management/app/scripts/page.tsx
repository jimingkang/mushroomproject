import Link from "next/link"
import { BatteryCharging, Settings, FileText, Plus, Play, Terminal, CheckCircle, XCircle } from "lucide-react"

import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { ScriptList } from "@/components/script-list"
import { ScriptExecutor } from "@/components/script-executor"

export default function ScriptsPage() {
  return (
    <div className="flex min-h-screen w-full flex-col">
      <header className="sticky top-0 z-10 flex h-16 items-center gap-4 border-b bg-background px-4 md:px-6">
        <Link href="/" className="flex items-center gap-2 font-semibold">
          <BatteryCharging className="h-6 w-6" />
          <span>RoboManager</span>
        </Link>
        <nav className="ml-auto flex gap-2">
          <Button variant="outline" size="sm" asChild>
            <Link href="/logs">
              <FileText className="mr-2 h-4 w-4" />
              Logs
            </Link>
          </Button>
          <Button variant="outline" size="sm" asChild>
            <Link href="/settings">
              <Settings className="mr-2 h-4 w-4" />
              Settings
            </Link>
          </Button>
        </nav>
      </header>
      <div className="flex items-center gap-4 border-b bg-background px-4 py-4 md:px-6">
        <div>
          <h1 className="text-xl font-bold tracking-tight">Script Management</h1>
          <p className="text-muted-foreground">Manage and execute Linux scripts remotely</p>
        </div>
        <div className="ml-auto flex gap-2">
          <Button size="sm" asChild>
            <Link href="/scripts/new">
              <Plus className="mr-2 h-4 w-4" />
              Add Script
            </Link>
          </Button>
        </div>
      </div>
      <main className="flex flex-1 flex-col gap-4 p-4 md:gap-8 md:p-8">
        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Total Scripts</CardTitle>
              <Terminal className="h-4 w-4 text-muted-foreground" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">24</div>
              <p className="text-xs text-muted-foreground">Available scripts</p>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Running</CardTitle>
              <Play className="h-4 w-4 text-green-500" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold text-green-600">3</div>
              <p className="text-xs text-muted-foreground">Currently executing</p>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Completed Today</CardTitle>
              <CheckCircle className="h-4 w-4 text-blue-500" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">47</div>
              <p className="text-xs text-muted-foreground">Successful executions</p>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Failed</CardTitle>
              <XCircle className="h-4 w-4 text-destructive" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold text-destructive">2</div>
              <p className="text-xs text-muted-foreground">Requires attention</p>
            </CardContent>
          </Card>
        </div>

        <Tabs defaultValue="scripts">
          <TabsList>
            <TabsTrigger value="scripts">Script Library</TabsTrigger>
            <TabsTrigger value="executor">Script Executor</TabsTrigger>
            <TabsTrigger value="history">Execution History</TabsTrigger>
          </TabsList>
          <TabsContent value="scripts" className="mt-4">
            <ScriptList />
          </TabsContent>
          <TabsContent value="executor" className="mt-4">
            <ScriptExecutor />
          </TabsContent>
          <TabsContent value="history" className="mt-4">
            <Card>
              <CardHeader>
                <CardTitle>Execution History</CardTitle>
                <CardDescription>Recent script execution results</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  {[
                    {
                      script: "backup_database.sh",
                      status: "completed",
                      startTime: "2025-01-17 14:00:00",
                      duration: "2m 34s",
                      exitCode: 0,
                    },
                    {
                      script: "system_update.sh",
                      status: "running",
                      startTime: "2025-01-17 14:15:00",
                      duration: "5m 12s",
                      exitCode: null,
                    },
                    {
                      script: "cleanup_logs.sh",
                      status: "failed",
                      startTime: "2025-01-17 13:45:00",
                      duration: "0m 15s",
                      exitCode: 1,
                    },
                  ].map((execution, index) => (
                    <div key={index} className="flex items-center justify-between p-3 border rounded-lg">
                      <div className="flex items-center gap-3">
                        {execution.status === "completed" && <CheckCircle className="h-4 w-4 text-green-500" />}
                        {execution.status === "running" && <Play className="h-4 w-4 text-blue-500" />}
                        {execution.status === "failed" && <XCircle className="h-4 w-4 text-destructive" />}
                        <div>
                          <p className="font-medium">{execution.script}</p>
                          <p className="text-sm text-muted-foreground">
                            Started: {execution.startTime} • Duration: {execution.duration}
                            {execution.exitCode !== null && ` • Exit Code: ${execution.exitCode}`}
                          </p>
                        </div>
                      </div>
                      <Button variant="outline" size="sm">
                        View Details
                      </Button>
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </main>
    </div>
  )
}
