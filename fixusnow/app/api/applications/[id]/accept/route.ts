import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { prisma } from "@/lib/prisma";

export async function POST(_req: NextRequest, { params }: { params: Promise<{ id: string }> }) {
  const session = await auth();
  if (!session?.user) return NextResponse.json({ error: "Unauthorized" }, { status: 401 });

  const user = session.user as { id: string; role: string };
  const { id } = await params;

  const application = await prisma.application.findUnique({
    where: { id },
    include: { task: true },
  });

  if (!application) return NextResponse.json({ error: "Not found" }, { status: 404 });
  if (application.task.customerId !== user.id) {
    return NextResponse.json({ error: "Forbidden" }, { status: 403 });
  }

  await prisma.$transaction([
    prisma.application.update({ where: { id }, data: { status: "ACCEPTED" } }),
    prisma.application.updateMany({
      where: { taskId: application.taskId, id: { not: id } },
      data: { status: "REJECTED" },
    }),
    prisma.task.update({
      where: { id: application.taskId },
      data: { status: "IN_PROGRESS", providerId: application.providerId },
    }),
  ]);

  return NextResponse.json({ success: true });
}
