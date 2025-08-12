import { CheckCircle2, Clock, XCircle } from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table"

interface TaskListProps {
  robotId: string
}

export function TaskList({ robotId }: TaskListProps) {
  // In a real app, you would fetch tasks based on the robot ID
  const tasks = [
    {
      id: "T-1001",
      name: "Assembly of Component X-42",
      status: "completed",
      priority: "high",
      assignedAt: "2025-07-09 08:30",
      completedAt: "2025-07-09 10:15",
    },
    {
      id: "T-1002",
      name: "Transport materials to Station B",
      status: "in-progress",
      priority: "medium",
      assignedAt: "2025-07-10 09:00",
      completedAt: null,
    },
    {
      id: "T-1003",
      name: "Quality inspection on Line C",
      status: "pending",
      priority: "low",
      assignedAt: "2025-07-10 14:00",
      completedAt: null,
    },
    {
      id: "T-1004",
      name: "Maintenance check on conveyor belt",
      status: "failed",
      priority: "high",
      assignedAt: "2025-07-08 11:30",
      completedAt: "2025-07-08 12:45",
    },
    {
      id: "T-1005",
      name: "Clean work area in Section D",
      status: "completed",
      priority: "low",
      assignedAt: "2025-07-09 16:00",
      completedAt: "2025-07-09 16:45",
    },
  ]

  return (
    <div className="rounded-md border">
      <Table>
        <TableHeader>
          <TableRow>
            <TableHead>Task ID</TableHead>
            <TableHead>Name</TableHead>
            <TableHead>Status</TableHead>
            <TableHead>Priority</TableHead>
            <TableHead>Assigned At</TableHead>
            <TableHead>Completed At</TableHead>
          </TableRow>
        </TableHeader>
        <TableBody>
          {tasks.map((task) => (
            <TableRow key={task.id}>
              <TableCell className="font-medium">{task.id}</TableCell>
              <TableCell>{task.name}</TableCell>
              <TableCell>
                <Badge
                  variant={
                    task.status === "completed"
                      ? "default"
                      : task.status === "in-progress"
                        ? "outline"
                        : task.status === "pending"
                          ? "secondary"
                          : "destructive"
                  }
                  className="capitalize"
                >
                  {task.status === "completed" && <CheckCircle2 className="mr-1 h-3 w-3" />}
                  {task.status === "in-progress" && <Clock className="mr-1 h-3 w-3" />}
                  {task.status === "failed" && <XCircle className="mr-1 h-3 w-3" />}
                  {task.status.replace("-", " ")}
                </Badge>
              </TableCell>
              <TableCell>
                <Badge
                  variant="outline"
                  className={
                    task.priority === "high"
                      ? "border-red-500 text-red-500"
                      : task.priority === "medium"
                        ? "border-yellow-500 text-yellow-500"
                        : "border-green-500 text-green-500"
                  }
                >
                  {task.priority}
                </Badge>
              </TableCell>
              <TableCell>{task.assignedAt}</TableCell>
              <TableCell>{task.completedAt || "-"}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </div>
  )
}
