import { type NextRequest, NextResponse } from "next/server"

export async function DELETE(request: NextRequest, { params }: { params: { id: string } }) {
  try {
    const scriptId = params.id

    // 这里应该从数据库中删除脚本
    // 目前只是模拟响应

    return NextResponse.json({
      success: true,
      message: `Script ${scriptId} deleted successfully`,
    })
  } catch (error) {
    console.error("Delete script error:", error)
    return NextResponse.json({ success: false, error: "Internal server error" }, { status: 500 })
  }
}

export async function PUT(request: NextRequest, { params }: { params: { id: string } }) {
  try {
    const scriptId = params.id
    const updates = await request.json()

    // 这里应该更新数据库中的脚本
    // 目前只是模拟响应

    return NextResponse.json({
      success: true,
      message: `Script ${scriptId} updated successfully`,
      script: { id: scriptId, ...updates },
    })
  } catch (error) {
    console.error("Update script error:", error)
    return NextResponse.json({ success: false, error: "Internal server error" }, { status: 500 })
  }
}
