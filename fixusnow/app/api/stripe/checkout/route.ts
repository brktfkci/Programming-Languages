import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { stripe } from "@/lib/stripe";

export async function POST(req: NextRequest) {
  const session = await auth();
  if (!session?.user) return NextResponse.json({ error: "Unauthorized" }, { status: 401 });

  const { taskId } = await req.json();

  const task = await prisma.task.findUnique({
    where: { id: taskId },
    include: { applications: { where: { status: "ACCEPTED" } } },
  });

  if (!task) return NextResponse.json({ error: "Task not found" }, { status: 404 });

  const accepted = task.applications[0];
  if (!accepted) return NextResponse.json({ error: "No accepted application" }, { status: 400 });

  const amount = Math.round(accepted.price * 100);

  const checkoutSession = await stripe.checkout.sessions.create({
    mode: "payment",
    payment_method_types: ["card"],
    line_items: [
      {
        price_data: {
          currency: "usd",
          unit_amount: amount,
          product_data: {
            name: task.title,
            description: task.description,
          },
        },
        quantity: 1,
      },
    ],
    success_url: `${process.env.NEXTAUTH_URL}/tasks/${taskId}?payment=success`,
    cancel_url: `${process.env.NEXTAUTH_URL}/tasks/${taskId}?payment=cancelled`,
    metadata: { taskId, applicationId: accepted.id },
  });

  await prisma.payment.upsert({
    where: { taskId },
    update: { stripeCheckoutSessionId: checkoutSession.id, amount: accepted.price },
    create: {
      taskId,
      amount: accepted.price,
      stripeCheckoutSessionId: checkoutSession.id,
    },
  });

  return NextResponse.json({ url: checkoutSession.url });
}
